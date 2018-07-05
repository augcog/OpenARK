#include "stdafx.h"
#include "Version.h"
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
        ASSERT(captureInterrupt == true, "beginCapture: already capturing from this camera");
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
        // initialize back buffers
        initializeImages();

        // call update with back buffer images (to allow continued operation on front end)
        update(xyzMapBuf, rgbMapBuf, irMapBuf, ampMapBuf, flagMapBuf);

        if (!badInput() && xyzMapBuf.data) {
            if (removeNoise) {
                this->removeNoise(xyzMapBuf, ampMapBuf, flagMapConfidenceThreshold());
            }
        }

        // lock all buffers while swapping
        std::lock_guard<std::mutex> lock(imageMutex);

        // when update is done, swap buffers to front
        swapBuffers();

        // call callbacks
        for (auto callback : updateCallbacks) {
            callback.second(*this);
        }

        return !badInput();
    }


    /** Returns default detection parameters for this depth camera class */
    DetectionParams::Ptr DepthCamera::getDefaultParams() const {
        return DetectionParams::DEFAULT;
    }

    /** Returns true on bad input */
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

    int DepthCamera::addUpdateCallback(std::function<void(DepthCamera&)> func)
    {
        int id;
        if (updateCallbacks.empty()) {
            id = 0;
        }
        else {
            id = updateCallbacks.rbegin()->first + 1;
        }

        updateCallbacks[id] = func;
        return id;
    }

    void DepthCamera::removeUpdateCallback(int id)
    {
        updateCallbacks.erase(id);
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
        xyzMapBuf.release();
        xyzMapBuf.create(sz, CV_32FC3);

        if (hasRGBMap()) {
            rgbMapBuf.release();
            rgbMapBuf.create(sz, CV_8UC3);
        }

        if (hasIRMap()) {
            irMapBuf.release();
            irMapBuf.create(sz, CV_8U);
        }

        if (hasAmpMap()) {
            ampMapBuf.release();
            ampMapBuf.create(sz, CV_32F);
        }

        if (hasFlagMap()) {
            flagMapBuf.release();
            flagMapBuf.create(sz, CV_8U);
        }
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
        std::lock_guard<std::mutex> lock(imageMutex);

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

        std::lock_guard<std::mutex> lock(imageMutex);

        fs["xyzMap"] >> xyzMap;
        fs["ampMap"] >> ampMap;
        fs["flagMap"] >> flagMap;
        fs["rgbMap"] >> rgbMap;
        fs["irMap"] >> irMap;
        fs.release();

        // call callbacks
        for (auto callback : updateCallbacks) {
            callback.second(*this);
        }

        return !(xyzMap.rows == 0 || ampMap.rows == 0 || flagMap.rows == 0);
    }

    const cv::Mat DepthCamera::getXYZMap() const
    {
        std::lock_guard<std::mutex> lock(imageMutex);
        if (xyzMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32FC3);
        return xyzMap;
    }

    const cv::Mat DepthCamera::getAmpMap() const
    {
        if (!hasAmpMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (ampMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_32F);
        return ampMap;
    }

    const cv::Mat DepthCamera::getFlagMap() const
    {
        if (!hasFlagMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (flagMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8U);
        return flagMap;
    }

    const cv::Mat DepthCamera::getRGBMap() const {
        if (!hasRGBMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (rgbMap.data == nullptr) return cv::Mat::zeros(getHeight(), getWidth(), CV_8UC3);
        return rgbMap;
    }

    const cv::Mat DepthCamera::getIRMap() const
    {
        if (!hasIRMap()) throw;

        std::lock_guard<std::mutex> lock(imageMutex);
        if (irMap.data == nullptr) return cv::Mat::zeros(getImageSize(), CV_8U);
        return irMap;
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
        using namespace std::chrono;
        steady_clock::time_point lastTime;
        steady_clock::time_point currTime;
        float timePerFrame;
        if (fps_cap > 0) {
            timePerFrame = 1e9f / fps_cap;
            lastTime = steady_clock::now();
        }

        while (interrupt == nullptr || !(*interrupt)) {
            this->nextFrame(remove_noise);

            // cap FPS
            if (fps_cap > 0) {
                currTime = steady_clock::now();
                steady_clock::duration delta = duration_cast<microseconds>(currTime - lastTime);

                if (delta.count() < timePerFrame) {
                    long long ms = (long long)(timePerFrame - delta.count()) / 1e6f;
                    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
                }
                lastTime = currTime;
            }
        }
    }
}
