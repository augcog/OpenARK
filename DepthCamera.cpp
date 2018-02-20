#include "stdafx.h"
#include "version.h"
#include "DepthCamera.h"
#include "Hand.h"
#include "FrameObject.h"

namespace ark {

    /**
     * Minimum depth of points (in meters). Points under this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_LOW = 0.14;

    /**
     * Maximum depth of points (in meters). Points above this depth are presumed to be noise. (0.0 to disable)
     */
    const float DepthCamera::NOISE_FILTER_HIGH = 0.99;

    DepthCamera::~DepthCamera()
    {
        badInputFlag = true;
        endCapture();
    }

    void DepthCamera::beginCapture(int fps_cap, bool remove_noise)
    {
        assert(captureInterrupt == true);
        captureInterrupt = false;
        std::thread thd(&DepthCamera::captureThreadingHelper, this, fps_cap,
            &captureInterrupt, remove_noise);
        thd.detach();
    }

    void DepthCamera::endCapture()
    {
        captureInterrupt = true;
    }

    bool DepthCamera::nextFrame(bool removeNoise)
    {
        {
            // lock back buffers
            std::lock_guard<std::mutex> backLockI(backMutex);

            // initialize back buffers
            initializeImages();

            // call update with back buffer images (to allow continued operation on front end)
            update(xyzMapBuf, rgbMapBuf, irMapBuf, ampMapBuf, flagMapBuf);

            if (!badInput() && xyzMapBuf.data) {
                if (removeNoise) {
                    this->removeNoise(xyzMapBuf, ampMapBuf, flagMapConfidenceThreshold());
                }
            }
        }

        // lock all buffers while swapping
        std::lock(frontMutex, backMutex, cacheMutex);
        std::lock_guard<std::mutex> frontLock(frontMutex, std::adopt_lock), 
            backLock(backMutex, std::adopt_lock), cacheLock(cacheMutex, std::adopt_lock);

        // when update is done, swap buffers to front
        swapBuffers();

        // clear cache flag
        isCached = 0;
        return !badInput();
    }

    void DepthCamera::computeNormalMap(const ObjectParams * params) {
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        const cv::Mat xyzMap = getXYZMap();

        {
            std::lock_guard<std::mutex> frontLock(frontMutex);
            util::computeNormalMap(xyzMap, normalMap, 4, params->normalResolution, false);
        }

        std::lock_guard<std::mutex> cacheLock(cacheMutex);
        isCached |= FLAG_NORMALS;
    }

    std::vector<HandPtr> & DepthCamera::getFrameHands(const ObjectParams * params, bool elim_planes)
    {
        {
            std::lock_guard<std::mutex> cacheLockI(cacheMutex);
            if (!this->xyzMap.data || (isCached & FLAG_FRM_HANDS)) return hands;
        }

        hands.clear();

        // 1. initialize
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        cv::Mat xyzMap = getXYZMap();
        const int R = xyzMap.rows, C = xyzMap.cols;

        /* records clusters pending reconsideration on the second pass (plane removal).
           each cluster's points are recorded in a different color. (only used if elim_planes enabled) */
        cv::Mat pendingClusters;
        if (elim_planes) pendingClusters = cv::Mat::zeros(R, C, CV_8U);

        float bestHandDist = FLT_MAX;

        // 2. eliminate large planes

        const int DOMINANT_PLANE_MIN_POINTS = params->dominantPlaneMinPoints * R * C /
                                        (params->normalResolution * params->normalResolution);

        if (elim_planes){
            std::vector<FramePlanePtr> planes = getFramePlanes(params);
            if (planes.size()) {
                for (auto plane : planes) {
                    if (plane->getPointsIJ().size() > DOMINANT_PLANE_MIN_POINTS) {
                        plane->cutFromXYZMap(xyzMap, params->handPlaneMinNorm);
                    }
                }
            }
        }

        // 3. first pass: try to detect hands directly
        // number of 'pending' clusters
        uchar pendingClusterCount = 0;
        boost::shared_ptr<Hand> bestHandObject =
            detectHandHelper(xyzMap, hands, params, &bestHandDist, &pendingClusters, 
                &pendingClusterCount);

        // 4. second pass: eliminate planes and try again
        if (elim_planes) {
            cv::Vec3b color;

            for (uchar k = 0; k < pendingClusterCount; ++k) {
                color = util::paletteColor(k);

                uchar maskColor = 255 - k;

                std::vector<Vec3f> equations;
                Vec_VecP2i points;
                Vec_VecV3f pointsXYZ;

                detectPlaneHelper(xyzMap, equations, points, pointsXYZ, params, nullptr,
                    &pendingClusters, maskColor);

                if (equations.size()) {
                    for (uint i = 0; i < equations.size(); ++i) {
                        util::removePlane(xyzMap, equations[i], params->handPlaneMinNorm);
                    }

                    bestHandObject = detectHandHelper(xyzMap, hands, params, 
                        &bestHandDist, nullptr, nullptr, &pendingClusters, maskColor);
                }
            }
        }

        if (bestHandObject != nullptr) {
            hands.push_back(bestHandObject);
        }

        std::lock_guard<std::mutex> cacheLock(cacheMutex);
        isCached |= FLAG_FRM_HANDS;
        return hands;
    }

    std::vector<FramePlanePtr> & DepthCamera::getFramePlanes(const ObjectParams * params)
    {
        {
            std::lock_guard<std::mutex> cacheLockI(cacheMutex);
            if (!this->xyzMap.data || (isCached & FLAG_FRM_PLANES)) return framePlanes;
        }

        framePlanes.clear();
        cv::Mat xyzMap = getXYZMap();

        std::vector<Vec3f> equations;
        Vec_VecP2i points;
        Vec_VecV3f pointsXYZ;

        detectPlaneHelper(xyzMap, equations, points, pointsXYZ, params);

        for (uint i = 0; i < equations.size(); ++i) {
            auto planePtr = boost::make_shared<FramePlane>
                (equations[i], points[i], pointsXYZ[i], xyzMap, params);

            if (planePtr->getSurfArea() > params->planeMinArea) {
                framePlanes.emplace_back(planePtr);
            }
        }

        if (params == nullptr) params = &ObjectParams::DEFAULT;

#ifdef DEBUG
        cv::Mat planeDebugVisual =
            cv::Mat::zeros(xyzMap.size() / params->normalResolution, CV_8UC3);

        for (int i = 0; i < framePlanes.size(); ++i) {
            Vec3b color = util::paletteColor(i);
            const std::vector<Point2i> & points = framePlanes[i]->getPointsIJ();

            for (uint j = 0; j < points.size(); ++j) {
                planeDebugVisual.at<Vec3b>(points[j] / params->normalResolution) = color;
            }
        }

        cv::resize(planeDebugVisual, planeDebugVisual, planeDebugVisual.size() * params->normalResolution,
            0, 0, cv::INTER_NEAREST);


        for (int i = 0; i < framePlanes.size(); ++i) {
            cv::putText(planeDebugVisual, std::to_string(framePlanes[i]->getSurfArea()), framePlanes[i]->getCenterIJ(), 0, 0.5, cv::Scalar(255, 255, 255));
        }

        cv::imshow("[Plane Debug]", planeDebugVisual);
#endif

        // cache planes for current frame
        std::lock_guard<std::mutex> cacheLock(cacheMutex);
        isCached |= FLAG_FRM_PLANES;
        return framePlanes;
    }

    std::vector<FrameObjectPtr> & DepthCamera::getFrameObjects(const ObjectParams * params)
    {        
        {
            std::lock_guard<std::mutex> cacheLockI(cacheMutex);
            if (!this->xyzMap.data || (isCached & FLAG_FRM_OBJS)) return frameObjects;
        }
        frameObjects.clear();

        // default parameters
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        // get planes and hands
        getFramePlanes(params); getFrameHands(params);

        // push resulting objects
        for (auto plane : framePlanes) frameObjects.push_back(plane);
        for (auto hand : hands) frameObjects.push_back(hand);

        // cache for current frame
        std::lock_guard<std::mutex> cacheLock(cacheMutex);
        isCached |= FLAG_FRM_OBJS;
        return frameObjects;
    }

    bool DepthCamera::badInput()
    {
        return badInputFlag;
    }

    /**
    Remove noise on zMap and xyzMap
    */
    void DepthCamera::removeNoise(cv::Mat & xyz_map, cv::Mat & amp_map, float confidence_thresh)
    {
        for (int r = 0; r < xyz_map.rows; ++r)
        {
            Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

            const float * ampptr = nullptr;
            if (amp_map.data) ampptr = amp_map.ptr<float>(r);

            for (int c = 0; c < xyz_map.cols; ++c)
            {
                if (ptr[c][2] > 0.0f) {
                    if (ptr[c][2] < NOISE_FILTER_LOW &&
                        (ptr[c][2] > NOISE_FILTER_HIGH || ptr[c][2] == 0.0) &&
                        (ampptr == nullptr || amp_map.data == nullptr ||
                            ampptr[c] < confidence_thresh)) {
                        ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
                    }
                }
            }
        }
    }

    bool DepthCamera::isCapturing()
    {
        return !captureInterrupt;
    }

    cv::Size DepthCamera::getImageSize() const
    {
        return cv::Size(getWidth(), getHeight());
    }


    const std::string DepthCamera::getModelName() const {
        return "DepthCamera";
    }

    void DepthCamera::initializeImages()
    {
        cv::Size sz = getImageSize();

        // initialize back buffers, if necessary
        if (xyzMapBuf.data == nullptr) 
            xyzMapBuf = cv::Mat::zeros(sz, CV_32FC3);

        if (rgbMapBuf.data == nullptr && hasRGBMap()) 
            rgbMapBuf = cv::Mat::zeros(sz, CV_8UC3);

        if (irMapBuf.data == nullptr && hasIRMap()) 
            irMapBuf = cv::Mat::zeros(sz, CV_8U);

        if (ampMapBuf.data == nullptr && hasAmpMap()) 
            ampMapBuf = cv::Mat::zeros(sz, CV_32F);

        if (flagMapBuf.data == nullptr && hasFlagMap()) 
            flagMapBuf = cv::Mat::zeros(sz, CV_8U);
    }

    /** swap a single buffer */
    void DepthCamera::swapBuffer(bool (DepthCamera::* check_func)() const, cv::Mat & img, cv::Mat & buf)
    {
        if ((this->*check_func)()) {
            cv::swap(img, buf);
        }
        else {
            img.data = nullptr;
        }
    }

    /** swap all buffers */
    void DepthCamera::swapBuffers()
    {
        cv::swap(xyzMap, xyzMapBuf);
        swapBuffer(&DepthCamera::hasRGBMap, rgbMap, rgbMapBuf);
        swapBuffer(&DepthCamera::hasIRMap, irMap, irMapBuf);
        swapBuffer(&DepthCamera::hasAmpMap, ampMap, ampMapBuf);
        swapBuffer(&DepthCamera::hasFlagMap, flagMap, flagMapBuf);
    }

    /**
    write a frame into file located at "destination"
    */
    bool DepthCamera::writeImage(std::string destination) const
    {
        cv::FileStorage fs(destination, cv::FileStorage::WRITE);
        std::lock_guard<std::mutex> frontLock(frontMutex);

        fs << "xyzMap" << xyzMap;
        fs << "ampMap" << ampMap;
        fs << "flagMap" << flagMap;
        fs << "rgbMap" << rgbMap;
        fs << "irMap" << irMap;

        fs.release();
        return true;
    }

    /**
    Reads a frame from file located at "source"
    */
    bool DepthCamera::readImage(std::string source)
    {
        cv::FileStorage fs;
        fs.open(source, cv::FileStorage::READ);

        std::lock(frontMutex, backMutex, cacheMutex);
        std::lock_guard<std::mutex> frontLock(frontMutex, std::adopt_lock),
            backLock(backMutex, std::adopt_lock), cacheLock(cacheMutex, std::adopt_lock);
        initializeImages();

        xyzMapBuf = cv::Mat(getHeight(), getWidth(), CV_32FC3);

        fs["xyzMap"] >> xyzMapBuf;
        fs["ampMap"] >> ampMapBuf;
        fs["flagMap"] >> flagMapBuf;
        fs["rgbMap"] >> rgbMapBuf;
        fs["irMap"] >> irMapBuf;
        fs.release();

        swapBuffers();
        isCached = 0;

        if (xyzMap.rows == 0 || ampMap.rows == 0 || flagMap.rows == 0)
        {
            return false;
        }

        return true;
    }

    const cv::Mat DepthCamera::getXYZMap() const
    {
        std::lock_guard<std::mutex> frontLock(frontMutex);
        if (xyzMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32FC3);
        return xyzMap.clone();
    }

    const cv::Mat DepthCamera::getAmpMap() const
    {
        if (!hasAmpMap()) throw;

        std::lock_guard<std::mutex> frontLock(frontMutex);
        if (ampMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32F);
        return ampMap.clone();
    }

    const cv::Mat DepthCamera::getFlagMap() const
    {
        if (!hasFlagMap()) throw;

        std::lock_guard<std::mutex> frontLock(frontMutex);
        if (flagMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8U);
        return flagMap.clone();
    }

    const cv::Mat DepthCamera::getRGBMap() const {
        if (!hasRGBMap()) throw;

        std::lock_guard<std::mutex> frontLock(frontMutex);
        if (rgbMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8UC3);
        return rgbMap.clone();
    }

    const cv::Mat DepthCamera::getIRMap() const
    {
        if (!hasIRMap()) throw;

        std::lock_guard<std::mutex> frontLock(frontMutex);
        if (irMap.data == nullptr) return cv::Mat::zeros(getImageSize(), CV_8U);
        return irMap.clone();
    }

    const cv::Mat DepthCamera::getNormalMap()
    {
        bool recompute = false;
        {
            // detect if we need to recompute
            std::lock_guard<std::mutex> cacheLock(cacheMutex);
            recompute = ( !(isCached & FLAG_NORMALS) || normalMap.data == nullptr) &&
                            xyzMap.data != nullptr;
        }

        if (recompute) {
            computeNormalMap();
        }

        std::lock_guard<std::mutex> frontLock(frontMutex);
        return normalMap.clone();
    }

    bool DepthCamera::hasAmpMap() const
    {
        // Assume no amp map, unless overridden
        return false;
    }

    bool DepthCamera::hasFlagMap() const
    {
        // Assume no flag map, unless overridden
        return false;
    }

    bool DepthCamera::hasRGBMap() const {
        // Assume no RGB image, unless overridden
        return false;
    }

    bool DepthCamera::hasIRMap() const
    {
        // Assume no IR image, unless overridden
        return false;
    }

    // note: depth camera must have XYZ map

    int DepthCamera::ampMapInvalidFlagValue() const{
        return -1;
    }

    float DepthCamera::flagMapConfidenceThreshold() const{
        return 0.5;
    }

    void DepthCamera::captureThreadingHelper(int fps_cap, volatile bool * interrupt, bool remove_noise)
    {
        unsigned long long lastTime =
            boost::chrono::system_clock::now().time_since_epoch() / boost::chrono::milliseconds(1);
        unsigned long long curTime;
        unsigned long long timePerFrame = 1000 / fps_cap;

        while (interrupt == nullptr || !(*interrupt)) {
            this->nextFrame(remove_noise);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

            // cap FPS
            curTime =
                boost::chrono::system_clock::now().time_since_epoch() / boost::chrono::milliseconds(1);

            unsigned long long delta = curTime - lastTime;

            if (delta < timePerFrame) {
                long long ms = (long long)((timePerFrame - delta) * 00000);
                boost::this_thread::sleep_for(boost::chrono::milliseconds(ms));
            }

            lastTime = curTime;
        }
    }

    void DepthCamera::detectPlaneHelper(const cv::Mat & xyz_map,
        std::vector<Vec3f> & output_equations, 
        Vec_VecP2i & output_points, Vec_VecV3f & output_points_xyz, 
        const ObjectParams * params, const cv::Mat * normal_map, const cv::Mat * fill_mask, 
        uchar fill_color)
    { 
        if (params == nullptr) params = &ObjectParams::DEFAULT;

        // 1. initialize
        cv::Mat tmpMap;

        cv::Mat floodFillMap; 

        if (normal_map == nullptr) {
            floodFillMap = getNormalMap();
        }
        else {
            floodFillMap = *normal_map;
        }

        int compId = -1;

        std::vector<Point2i> allIndices(xyz_map.rows * xyz_map.cols);

        // 2. find 'subplanes' i.e. all flat objects visible in frame and combine similar ones
        // stores points on each plane
        std::vector<boost::shared_ptr<std::vector<Point2i> > > planes;

        // stores points (in 3D coords) on each plane
        std::vector<boost::shared_ptr<std::vector<Vec3f> > > planesXyz;

        // equations of the planes: ax + by - z + c = 0
        std::vector<Vec3f> planeEquation;

        // compute constants
        const int SUBPLANE_MIN_POINTS =
            params->subplaneMinPoints * allIndices.size() /
            (params->normalResolution * params->normalResolution);

        const int PLANE_MIN_POINTS =
            params->planeMinPoints * allIndices.size() /
            (params->normalResolution * params->normalResolution);

        const int PLANE_MIN_INLIERS =
            params->planeEquationMinInliers * allIndices.size() /
            (params->normalResolution * params->normalResolution);

        for (int i = 0; i < normalMap.rows; i += params->normalResolution) {
            for (int j = 0; j < normalMap.cols; j += params->normalResolution) {
                Point2i pt(j, i);

                if (floodFillMap.at<Vec3f>(pt)[2] == 0) continue;

                // flood fill normals
                int numPts = util::floodFill(floodFillMap, pt, params->planeFloodFillThreshold,
                                             &allIndices, nullptr, nullptr,
                                             params->normalResolution, 0,
                                             fill_mask, fill_color, &floodFillMap);

                if (numPts >= SUBPLANE_MIN_POINTS) {
                    // filter out outliers & find plane equation
                    std::vector<Vec3f> allXyzPoints(numPts);

                    for (int k = 0; k < numPts; ++k) {
                        allXyzPoints[k] = xyz_map.at<Vec3f>(allIndices[k]);;
                    }

                    util::radixSortPoints(allIndices, floodFillMap.cols, floodFillMap.rows,
                        numPts, &allXyzPoints);
                    double surfArea = util::surfaceArea(floodFillMap.size(), allIndices,
                        allXyzPoints, numPts);

                    if (surfArea < params->subplaneMinArea) {
                        continue;
                    }

                    Vec3f eqn = util::linearRegression(allXyzPoints, numPts);

                    // combine close planes
                    uint i;

                    for (i = 0; i < planeEquation.size(); ++i) {
                        if (util::norm(planeEquation[i] - eqn) < params->planeCombineThreshold) {
                            // found similar plane
                            break;
                        }
                    }

                    boost::shared_ptr<std::vector<Point2i>> pointStore;
                    boost::shared_ptr<std::vector<Vec3f>> pointStoreXyz;

                    if (i >= planeEquation.size()) {
                        // no similar plane found
                        planeEquation.push_back(eqn);
                        planes.emplace_back(boost::make_shared<std::vector<Point2i> >());
                        planesXyz.emplace_back(boost::make_shared<std::vector<Vec3f> >());
                        pointStore = *planes.rbegin();
                        pointStoreXyz = *planesXyz.rbegin();
                    }
                    else {
                        // similar plane found
                        pointStore = planes[i];
                        pointStoreXyz = planesXyz[i];
                    }

                    // save plane points to store
                    int start = (int)pointStore->size();
                    pointStore->resize(start + numPts);
                    pointStoreXyz->resize(start + numPts);

                    for (int i = 0; i < numPts; ++i) {
                        (*pointStore)[start + i] = allIndices[i];
                        (*pointStoreXyz)[start + i] = allXyzPoints[i];
                    }
                }
            }
        }

        // 3. find equations of the combined planes and construct Plane objects with the data
        for (uint i = 0; i < planeEquation.size(); ++i) {
            int SZ = (int)planes[i]->size();
            if (SZ < PLANE_MIN_POINTS) continue;

            std::vector<Vec3f> pointsXYZ;
            util::removeOutliers(*planesXyz[i], pointsXYZ, params->planeOutlierRemovalThreshold);

            planeEquation[i] = util::linearRegression(pointsXYZ);

            int goodPts = 0;

            for (uint j = 0; j < SZ; ++j) {
                float norm = util::pointPlaneNorm((*planesXyz[i])[j], planeEquation[i]);
                if (norm < params->handPlaneMinNorm) {
                    ++goodPts;
                }
            }

            if (goodPts < PLANE_MIN_INLIERS) continue;

            // push to output
            output_points.push_back(planes[i]);
            output_points_xyz.push_back(planesXyz[i]);
            output_equations.push_back(planeEquation[i]);
        }
    }

    boost::shared_ptr<Hand> DepthCamera::detectHandHelper(const cv::Mat & xyz_map,
        std::vector<HandPtr> & output_hands, const ObjectParams * params, float * best_hand_dist,
        cv::Mat * pending_mask, uchar * pending_count, const cv::Mat * fill_mask, 
        uchar fill_color)
    {

        // 1. initialize
        uchar pendingClusterCount = 0;
        cv::Mat floodFillMap = xyz_map.clone();

        const int R = xyz_map.rows, C = xyz_map.cols;

        float closestHandDist = FLT_MAX;
        if (best_hand_dist) closestHandDist = *best_hand_dist;

        // 2. try to directly cluster objects using flood fill
        std::vector<Point2i> allIjPoints(R * C + 1);
        std::vector<Vec3f> allXyzPoints(R * C + 1);

        // compute the minimum number of points in a cluster according to params
        const int CLUSTER_MIN_POINTS = (int) (params->handClusterMinPoints * R * C);

        float bestHandDist = FLT_MAX;
        boost::shared_ptr<Hand> bestHandObject;

        for (int r = 0; r < floodFillMap.rows; r += params->handClusterInterval)
        {
            Vec3f * ptr = floodFillMap.ptr<Vec3f>(r);
            const uchar * maskPtr;
            if (fill_mask) maskPtr = fill_mask->ptr<uchar>(r);

            for (int c = 0; c < floodFillMap.cols; c += params->handClusterInterval)
            {
                if ((!fill_mask || maskPtr[c] == fill_color) && ptr[c][2] > 0)
                {
                    int points_in_comp = util::floodFill(floodFillMap, Point2i(c, r),
                        params->handClusterMaxDistance,
                        &allIjPoints, &allXyzPoints, nullptr, 1, 6, nullptr, 0, &floodFillMap);

                    if (points_in_comp >= CLUSTER_MIN_POINTS)
                    {
                        auto ijPoints = boost::make_shared<std::vector<Point2i> >
                            (allIjPoints.begin(), allIjPoints.begin() + points_in_comp);
                        auto xyzPoints = boost::make_shared<std::vector<Vec3f> >
                            (allXyzPoints.begin(), allXyzPoints.begin() + points_in_comp);

                        // if matching required conditions, construct 3D object
                        auto handPtr = boost::make_shared<Hand>(ijPoints, xyzPoints, xyz_map,
                                params, false, points_in_comp);

                        if (handPtr->isValidHand()) {
                            float distance = handPtr->getDepth();

                            if (distance < closestHandDist) {
                                bestHandObject = handPtr;
                                closestHandDist = distance;
                            }

                            if (handPtr->getSVMConfidence() >
                                params->handSVMHighConfidenceThresh ||
                                !params->handUseSVM) {
                                 // avoid duplicate hand
                                if (bestHandObject == handPtr) bestHandObject = nullptr;
                                output_hands.push_back(handPtr);
                            }
                        }

                        else if (pending_mask && pendingClusterCount < 254) {
                            // in the second pass (plane elimination), reconsider this cluster
                            for (int i = 0; i < ijPoints->size(); ++i) {
                                pending_mask->at<uchar>(ijPoints->at(i)) =
                                    255-pendingClusterCount;
                            }
                            ++pendingClusterCount;
                        }
                    }
                }
            }
        }

        if (pending_count) {
            *pending_count = pendingClusterCount;
        }

        if (best_hand_dist) {
            *best_hand_dist = closestHandDist;
        }

        return bestHandObject;
    }
}
