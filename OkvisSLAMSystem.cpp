#include "stdafx.h"
#include "OkvisSLAMSystem.h"

namespace ark {

    OkvisSLAMSystem::OkvisSLAMSystem(const std::string & strVocFile, const std::string & strSettingsFile) :
        start_(0.0), t_imu_(0.0), deltaT_(1.0), num_frames_(0), kill(false), 
        sparse_maps_(), active_map_index(-1), map_id_counter_(0), new_map_checker(false),map_timer(0),
        strVocFile(strVocFile), matcher_(nullptr), bowId_(0), lastLoopClosureTimestamp_(0) {

        okvis::VioParametersReader vio_parameters_reader;
        try {
            vio_parameters_reader.readConfigFile(strSettingsFile);
        }
        catch (okvis::VioParametersReader::Exception ex) {
            std::cerr << ex.what() << "\n";
            return;
        }

        //okvis::VioParameters parameters;
        vio_parameters_reader.getParameters(parameters_);

        setEnableLoopClosure(parameters_.loopClosureParameters.enabled,strVocFile,true, new brisk::BruteForceMatcher());
        createNewMap();

        //initialize Visual odometry
        okvis_estimator_ = std::make_shared<okvis::ThreadedKFVio>(parameters_);
        okvis_estimator_->setBlocking(true);

        //Okvis's outframe is our inframe
        auto frame_callback = [this](const okvis::Time& timestamp, okvis::OutFrameData::Ptr frame_data) {
            frame_data_queue_.enqueue({ frame_data,timestamp });
        };

        okvis_estimator_->setFrameCallback(frame_callback);

        //at thread to pull from queue and call our own callbacks
        frameConsumerThread_ = std::thread(&OkvisSLAMSystem::FrameConsumerLoop, this);

        frame_data_queue_.clear();
        frame_queue_.clear();
        std::cout << "SLAM Initialized\n";
    }

    void OkvisSLAMSystem::Start() {
        //Do nothing, starts automatically
    }

    void OkvisSLAMSystem::FrameConsumerLoop() {
        while (!kill) {
             
            //Get processed frame data from OKVIS
            StampedFrameData frame_data;
            while (!frame_data_queue_.try_dequeue(&frame_data)) {
                if (okvis_estimator_->isReset() && !new_map_checker) {
                    if(map_timer >= kMapCreationCooldownFrames_)
                    {
                        cout<<"Created new map"<<endl;
                        new_map_checker = true;
                        createNewMap();
                    }
                    map_timer=0;
                    
                } else if (!okvis_estimator_->isReset() && new_map_checker) {
                    frame_queue_.clear();
                    frame_data_queue_.clear();
                    new_map_checker = false;
                }
                if (okvis_estimator_->isReset()) {
                    frame_queue_.clear();
                    frame_data_queue_.clear();
                }
                else
                {
                    if(map_timer<=kMapCreationCooldownFrames_)
                        map_timer++;
                }
                
                if(kill)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            //Get the corresponding frame from queue
            WrappedMultiCameraFrame wrapped_frame;
            bool frame_found = false;
            do {
                if (!(frame_found = frame_queue_.try_dequeue(&wrapped_frame))) {
                    break;
                } 
            } while (okvis::Time(wrapped_frame.frame->timestamp_) < frame_data.timestamp);
            if (!frame_found){
                std::cout << "ERROR, FRAME NOT FOUND, THIS SHOULDN'T HAPPEN\n";
                continue;
            }

            //construct output frame
            MultiCameraFrame::Ptr out_frame = wrapped_frame.frame;
            out_frame->frameId_ = frame_data.data->id;
            out_frame->T_KS_ = frame_data.data->T_KS.T();
            out_frame->keyframeId_ = frame_data.data->keyframe_id;

            //add sensor transforms
            //Note: this could potentially just be done once for the system
            //Including here for now in case we switch to optimized results
            for (size_t i = 0; i <  wrapped_frame.frame->images_.size(); i++) {
                okvis::kinematics::Transformation T_SC;
                if (i < parameters_.nCameraSystem.numCameras()) {
                    T_SC = (*parameters_.nCameraSystem.T_SC(i));
                }
                else if (i - parameters_.nCameraSystem.numCameras() < parameters_.secondaryCameraSystem.numCameras()) {
                    T_SC = (*parameters_.secondaryCameraSystem.T_SC(i - parameters_.nCameraSystem.numCameras()));
                } else {
                    //no relationship data to imu
                    //TODO: throw errror
                    T_SC = okvis::kinematics::Transformation::Identity();
                }
                out_frame->T_SC_.push_back(T_SC.T());
            }

            bool loopClosureDetected = false;
            bool mapsMerged = false;
            int deleted_map_index = -1;
            //check if keyframe
            if(frame_data.data->is_keyframe){
                if(out_frame->keyframeId_!=out_frame->frameId_){
                    std::cout << "ERROR, KEYFRAME ID INCORRECT, THIS SHOULDN'T HAPPEN\n";
                    continue;
                }
                MapKeyFrame::Ptr keyframe = MapKeyFrame::Ptr(new MapKeyFrame);
                keyframe->frameId_=out_frame->frameId_;
                keyframe->T_WS_ = frame_data.data->T_WS.T();
                keyframe->T_SC_ = out_frame->T_SC_;
                keyframe->timestamp_ = out_frame->timestamp_;
                //copy keypoints and descriptors to output
                keyframe->keypoints_.resize(frame_data.data->keypoints.size());
                keyframe->keypoints3dh_C.resize(frame_data.data->keypoints.size());
                keyframe->descriptors_.resize(frame_data.data->descriptors.size());
                for(size_t cam_idx=0 ; cam_idx<frame_data.data->keypoints.size() ; cam_idx++){
                    keyframe->keypoints_[cam_idx].resize(frame_data.data->keypoints[cam_idx].size());  

                    keyframe->keypoints3dh_C[cam_idx].resize(frame_data.data->keypoints[cam_idx].size());
                    //get transform of the camera
                    for(int i=0; i<frame_data.data->keypoints[cam_idx].size(); i++){
                        //copy keypoint
                        keyframe->keypoints_[cam_idx][i] = frame_data.data->keypoints[cam_idx][i];
                        //get estimated 3d position of keypoint in current camera frame
                        cv::Vec3f pt3d = out_frame->images_[2].at<cv::Vec3f>(
                            std::round(keyframe->keypoints_[cam_idx][i].pt.y),
                            std::round(keyframe->keypoints_[cam_idx][i].pt.x));
                        if(pt3d[2] >0)
                            keyframe->keypoints3dh_C[cam_idx][i] = Eigen::Vector4d(pt3d[0],pt3d[1],pt3d[2],1);
                        else
                            keyframe->keypoints3dh_C[cam_idx][i] = Eigen::Vector4d(0,0,0,0);
                    }
                    keyframe->descriptors_[cam_idx]=frame_data.data->descriptors[cam_idx];
                }

                // apply correction
                keyframe->T_WS_ = correction_ * keyframe->T_WS_;
                MapKeyFrame::Ptr loop_kf = nullptr;
                Eigen::Affine3d transformEstimate;
                if (getActiveMap()->getNumKeyframes() >= kMinimumKeyframes_
                        && detectLoopClosure(keyframe, loop_kf, transformEstimate)) {
                    //cout << "Loop closure detected" << endl;
                    loopClosureDetected = true;
                    for (auto it = sparse_maps_.begin(); it != sparse_maps_.end(); it++) {
                        int mapId = it->first;
                        auto sparseMap = it->second;
                        if (mapId == active_map_index)
                            continue;
                        
                        if (sparseMap->getKeyframe(loop_kf->frameId_) != nullptr) {
                            cout << "MapMerge: maps " << mapId << " with " << active_map_index <<
                                    " and frames " << keyframe->frameId_ << " with " << loop_kf->frameId_ << endl;
                            auto mergedMap = mergeMaps(sparseMap, getActiveMap(), keyframe, loop_kf, transformEstimate);
                            if (mergedMap == sparseMap) {
                                sparse_maps_.erase(active_map_index);
                                deleted_map_index = active_map_index;
                                active_map_index = mapId;
                            } else {
                                sparse_maps_.erase(mapId);
                                deleted_map_index = mapId;
                            }
                            
                            mapsMerged = true;
                            break;
                        }
                    }
                }
                
                if (!mapsMerged) {
                    getActiveMap()->addKeyframe(keyframe, loop_kf, transformEstimate);
                }
            }

            out_frame->keyframe_ = getActiveMap()->getKeyframe(out_frame->keyframeId_);

            //Notify callbacks
            if (mapsMerged) {
                for (MapSparseMapMergeHandler::const_iterator callback_iter = mMapSparseMapMergeHandler.begin();
                        callback_iter != mMapSparseMapMergeHandler.end(); ++callback_iter) {
                    const MapSparseMapMergeHandler::value_type& pair = *callback_iter;
                    pair.second(deleted_map_index, active_map_index);
                }
            }

            if(frame_data.data->is_keyframe){
                for (MapKeyFrameAvailableHandler::const_iterator callback_iter = mMapKeyFrameAvailableHandler.begin();
                    callback_iter != mMapKeyFrameAvailableHandler.end(); ++callback_iter) {
                    const MapKeyFrameAvailableHandler::value_type& pair = *callback_iter;
                    pair.second(out_frame);
                }
            }

            for (MapFrameAvailableHandler::const_iterator callback_iter = mMapFrameAvailableHandler.begin();
                callback_iter != mMapFrameAvailableHandler.end(); ++callback_iter) {
                const MapFrameAvailableHandler::value_type& pair = *callback_iter;
                pair.second(out_frame);
            }

            if (loopClosureDetected) {
                for (MapLoopClosureDetectedHandler::const_iterator callback_iter = mMapLoopClosureHandler.begin();
                    callback_iter != mMapLoopClosureHandler.end(); ++callback_iter) {
                    const MapLoopClosureDetectedHandler::value_type& pair = *callback_iter;
                        pair.second();
                }
            }
        }
    }

    /*void OkvisSLAMSystem::PushFrame(const std::vector<cv::Mat>& images, const double &timestamp) {
        if (okvis_estimator_ == nullptr)
            return;
        okvis::Time t_image(timestamp / 1e9);
        if (start_ == okvis::Time(0.0)) {
            start_ = t_image;
        }

        if (t_image - start_ > deltaT_) {
            if(mMapFrameAvailableHandler.size()>0){
                frame_queue_.enqueue({ images,t_image,num_frames_ });
            }
            num_frames_++;
            for (size_t i = 0; i < images.size(); i++) {
                if (i < parameters_.nCameraSystem.numCameras()){
                    //printf("add image: %i\n", i);
                    okvis_estimator_->addImage(t_image, i, images[i]);
                }
            }
        }
    }

    void OkvisSLAMSystem::PushFrame(const cv::Mat image, const double &timestamp) {
        if (okvis_estimator_ == nullptr)
            return;
        okvis::Time t_image(timestamp / 1e9);
        if (start_ == okvis::Time(0.0)) {
            start_ = t_image;
        }

        if (t_image - start_ > deltaT_) {
            std::vector<cv::Mat> images;
            images.push_back(image);
            if(mMapFrameAvailableHandler.size()>0){
                frame_queue_.enqueue({ images,t_image,num_frames_ });
            }
            num_frames_++;
            okvis_estimator_->addImage(t_image, 0, image);
        }
    }*/

    void OkvisSLAMSystem::PushFrame(MultiCameraFrame::Ptr frame){
        if (okvis_estimator_ == nullptr)
            return;
        okvis::Time t_image(frame->timestamp_ / 1e9);
        if (start_ == okvis::Time(0.0)) {
            start_ = t_image;
        }

        if (t_image - start_ > deltaT_) {
            if(mMapFrameAvailableHandler.size()>0){
                frame_queue_.enqueue({ frame});
            }
            num_frames_++;
            for (size_t i = 0; i < frame->images_.size(); i++) {
                if (i < parameters_.nCameraSystem.numCameras()){
                    //printf("add image: %i\n", i);
                    okvis_estimator_->addImage(t_image, i, frame->images_[i]);
                }
            }
        }   
    }

    void OkvisSLAMSystem::PushIMU(const std::vector<ImuPair, Eigen::aligned_allocator<ImuPair>>& imu) {
        if (okvis_estimator_ == nullptr) return;
        for (size_t i = 0; i < imu.size(); i++) {
            PushIMU(imu[i]);
        }
    }

    void OkvisSLAMSystem::PushIMU(const ImuPair& imu) {
        if (okvis_estimator_ == nullptr) return;
        t_imu_ = okvis::Time(imu.timestamp / 1e9);
        if (t_imu_ - start_ + okvis::Duration(1.0) > deltaT_) {
            //std::cout << imu.timestamp / 1e9 << " , " << imu.accel.transpose() << " , " << imu.gyro.transpose() << std::endl;
            okvis_estimator_->addImuMeasurement(t_imu_, imu.accel, imu.gyro);
        }
    }

    void OkvisSLAMSystem::PushIMU(double timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d gyro) {
        if (okvis_estimator_ == nullptr) return;
        t_imu_ = okvis::Time(timestamp / 1e9);
        if (t_imu_ - start_ + okvis::Duration(1.0) > deltaT_) {
            okvis_estimator_->addImuMeasurement(t_imu_, accel, gyro);
        }
    }

    void OkvisSLAMSystem::createNewMap() {
        const auto newMap = std::make_shared<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>>();

        // delete current map if it's too small
        if (sparse_maps_.size() != 0 && getActiveMap()->getNumKeyframes() < kMinimumKeyframes_) {
            sparse_maps_.erase(active_map_index);
        }

        sparse_maps_[map_id_counter_] = newMap;
        active_map_index = map_id_counter_;
        map_id_counter_ ++;
        correction_ = Eigen::Matrix4d::Identity();
        for (MapSparseMapCreationHandler::const_iterator callback_iter = mMapSparseMapCreationHandler.begin();
            callback_iter != mMapSparseMapCreationHandler.end(); ++callback_iter) {
            const MapSparseMapCreationHandler::value_type& pair = *callback_iter;
            pair.second(active_map_index);
        }
    }

    void OkvisSLAMSystem::setEnableLoopClosure(bool enableUseLoopClosures = false, std::string vocabPath = "",
            bool binaryVocab = true, cv::DescriptorMatcher* matcher = nullptr) {
        matcher_.reset(matcher);
        useLoopClosures_ = enableUseLoopClosures;
        if(useLoopClosures_){
            std::cout << "Loading Vocabulary From: " << vocabPath << std::endl;
            vocab_.reset(new DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>()); 
            if(!binaryVocab){
                vocab_->load(vocabPath);
            }else{
                // Load old BRISK
                // vocab_->binaryLoad(vocabPath); //Note: Binary Loading only supported for ORB/BRISK vocabularies

                // Load customized ORB
                vocab->loadFromTextFile( vocabPath );
            }
            //vocab_->save(vocabPath+std::string(".tst"));
            std::cout << "Vocabulary Size: " << vocab_->size() << std::endl;
            typename DLoopDetector::TemplatedLoopDetector<DBoW2::FORB::TDescriptor, DBoW2::FORB>::Parameters detectorParams(0,0);
            //can change specific parameters if we want
            // Parameters given by default are:
            // use nss = true
            // alpha = 0.3
            // k = 3
            // geom checking = GEOM_DI
            detectorParams.k=2;
            detectorParams.di_levels = 4;
            detector_.reset(new DLoopDetector::TemplatedLoopDetector<DBoW2::FORB::TDescriptor, DBoW2::FORB>(*vocab_,detectorParams));
            std::cout << "Map Initialized\n";
        }
    }

    bool OkvisSLAMSystem::detectLoopClosure(MapKeyFrame::Ptr kf, MapKeyFrame::Ptr &loop_kf, Eigen::Affine3d &transformEstimate) {
        bool shouldDetectLoopClosure = kf->timestamp_-lastLoopClosureTimestamp_>0.2*1e9;
        if(!(useLoopClosures_ && shouldDetectLoopClosure)) {
            return false;
        }
        lastLoopClosureTimestamp_=kf->timestamp_;
        std::vector<cv::Mat> bowDesc;
        kf->descriptorsAsVec(0,bowDesc);
        DLoopDetector::DetectionResult result;
        auto local_keypoints = kf->keypoints(0);
        detector_->detectLoop(local_keypoints,bowDesc,result);
        if(result.detection()){
            loop_kf = bowFrameMap_[result.match];
        }else{
            //We only want to record a frame if it is not matched with another image
            //no need to duplicate
            bowFrameMap_[bowId_]=kf;
            bowId_++;
            return false; //pose added to graph, no loop detected, nothing left to do
        }

        //transform estimation
        //TODO: should move to function to be set as one of a variety of methods

        //brute force matching
        std::vector<cv::DMatch> matches; 
        //query,train
        matcher_->match(kf->descriptors(0),loop_kf->descriptors(0), matches);
        std::cout << "detectLoopClosure: " << "sizes: kf: " << kf->descriptors(0).rows << " loop_kf: "
            << loop_kf->descriptors(0).rows << " matches: " << matches.size() << " loop_kf id: " << loop_kf->frameId_ << std::endl;

        //get feature point clouds
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(int i=0; i<kf->keypoints(0).size(); i++){
            Eigen::Vector4d kp3dh_C = kf->homogeneousKeypoints3d(0)[i];
            kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
        } 
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr loop_kf_feat_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(int i=0; i<loop_kf->keypoints(0).size(); i++){
            Eigen::Vector4d kp3dh_C = loop_kf->homogeneousKeypoints3d(0)[i];
            loop_kf_feat_cloud->points.push_back(pcl::PointXYZ(kp3dh_C[0],kp3dh_C[1],kp3dh_C[2]));
        }

        //convert DMatch to correspondence
        std::vector<int> correspondences(matches.size());
        for(int i=0; i<matches.size(); i++){
            if(loop_kf->homogeneousKeypoints3d(0)[matches[i].queryIdx][3]!=0 && loop_kf->homogeneousKeypoints3d(0)[matches[i].trainIdx][3]!=0)
                correspondences[matches[i].queryIdx]=matches[i].trainIdx;
            else
                correspondences[matches[i].queryIdx]=-1;;
        }
        int numInliers;
        std::vector<bool> inliers;
        //find initial transform estimate using feature points
        CorrespondenceRansac<pcl::PointXYZ>::getInliersWithTransform(
                kf_feat_cloud, loop_kf_feat_cloud, correspondences,
                3, 0.2, 50, numInliers, inliers, transformEstimate);
        if(((float)numInliers)/correspondences.size()<0.3) {
            loop_kf = nullptr;
            return false; 
        }

        transformEstimate = PointCostSolver<pcl::PointXYZ>::solve(kf_feat_cloud,loop_kf_feat_cloud,
                                            correspondences, inliers, transformEstimate);

        //std::cout << transformEstimate.matrix() << std::endl;

        return true;
    }


    std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> OkvisSLAMSystem::mergeMaps(
            std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> olderMap,
            std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> currentMap, MapKeyFrame::Ptr kf,
            MapKeyFrame::Ptr loop_kf, Eigen::Affine3d &transformEstimate) {
    
        // Loop is detected. Merge smaller map into bigger map and save correction
        std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> mapA;
        std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> mapB;
        Eigen::Matrix4d correction;
        Eigen::Matrix4d kfCorrection;
        Eigen::Matrix4d T_KfKloop = kf->T_SC_[2]*transformEstimate.inverse().matrix()*kf->T_SC_[2].inverse();
        if (olderMap->getNumKeyframes() > currentMap->getNumKeyframes()) {
            //merge current map into older map
            mapA = currentMap;
            mapB = olderMap;
            correction = loop_kf->T_WS() * (kf->T_WS() * T_KfKloop).inverse();
            kfCorrection = correction;
            correction_ = correction * correction_;
            mapB->currentKeyframeId = mapA->currentKeyframeId;
        } else {
            //merge older map into current map
            mapA = olderMap;
            mapB = currentMap;
            correction = kf->T_WS() * T_KfKloop * loop_kf->T_WS().inverse();
            kfCorrection = Eigen::Matrix4d::Identity();
        }

        //adding keyframes from mapA to mapB
        for (auto framePair = mapA->frameMap_.begin(); framePair != mapA->frameMap_.end(); framePair++) {
            auto frame = framePair->second;
            frame->setOptimizedTransform(correction * frame->T_WS());
            frame->T_WS_ = correction * frame->T_WS_;
            mapB->frameMap_[frame->frameId_] = frame;
        }

        //adding constraints and poses to mapB's pose graph
        mapB->graph_.constraintMutex.lock();
        mapA->graph_.constraintMutex.lock();
        mapB->graph_.constraints_.insert(
            mapB->graph_.constraints_.end(),
            mapA->graph_.constraints_.begin(),
            mapA->graph_.constraints_.end()
        );
        mapA->graph_.constraintMutex.unlock();
        for (auto framePair = mapA->frameMap_.begin(); framePair != mapA->frameMap_.end(); framePair++) {
            mapB->graph_.poses_.insert(std::pair<int,GraphPose>(
                framePair->first,framePair->second->T_WS()));
        }
        mapB->graph_.constraintMutex.unlock();

        //adding current keyframe to mapB and optimizing pose graph
        kf->T_WS_ = kfCorrection * kf->T_WS_;
        mapB->addKeyframe(kf, loop_kf, transformEstimate);

        return mapB;
    }

    void OkvisSLAMSystem::display() {
        if (okvis_estimator_ == nullptr)
            return;
        okvis_estimator_->display();
    }

    void OkvisSLAMSystem::ShutDown()
    {
        frame_queue_.clear();
        frame_data_queue_.clear();
        kill=true;
        frameConsumerThread_.join();
        okvis_estimator_.reset();
    }

    OkvisSLAMSystem::~OkvisSLAMSystem() {
        frame_queue_.clear();
        frame_data_queue_.clear();
        kill=true;
    }

    void OkvisSLAMSystem::RequestStop()
    {
        okvis_estimator_ = nullptr;
    }

    bool OkvisSLAMSystem::IsRunning()
    {
        return okvis_estimator_ == nullptr;
    }

    void OkvisSLAMSystem::getActiveFrames(std::vector<int>& frame_ids){
        getActiveMap()->getFrames(frame_ids);
    }

    void OkvisSLAMSystem::getTrajectory(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut){
        getActiveMap()->getTrajectory(trajOut);
    }

    std::shared_ptr<SparseMap<DBoW2::FORB::TDescriptor, DBoW2::FORB>> OkvisSLAMSystem:: getActiveMap() {
        if (sparse_maps_.find(active_map_index) == sparse_maps_.end()) {
            std::cout << "Null map returned \n";
            return nullptr;
        } else {
            return sparse_maps_[active_map_index];
        }
    }

    void OkvisSLAMSystem::getMappedTrajectory(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& trajOut) {
        for (auto it = sparse_maps_.begin(); it != sparse_maps_.end(); it++) {
            it->second->getMappedTrajectory(frameIdOut, trajOut);
        }
    }
    

} //ark