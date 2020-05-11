#include "stdafx.h"
#include "OkvisSLAMSystem.h"

namespace ark {

    OkvisSLAMSystem::OkvisSLAMSystem(const std::string & strVocFile, const std::string & strSettingsFile) :
        start_(0.0), t_imu_(0.0), deltaT_(1.0), num_frames_(0), kill(false), 
        sparse_map_vector(), active_map_index(-1), new_map_checker(false),map_timer(0), strVocFile(strVocFile){

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
                    if(map_timer >= 15)
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
                    if(map_timer<=15)
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

            bool addKeyFrameResult = false;
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

                for (int i = 0; i < sparse_map_vector.size()-1; i++) {
                    const auto sparseMap = sparse_map_vector[i];
                    if (sparseMap->detectLoopClosure(keyframe)) {
                        active_map_index = i;
                        // delete all the new map after active map
                        sparse_map_vector.resize(active_map_index+1);
                        cout << "Found loop in old maps, deleting newer maps after: " << active_map_index << endl;
                        break;
                    }
                }

                addKeyFrameResult = getActiveMap()->addKeyframe(keyframe);

            }

            out_frame->keyframe_ = getActiveMap()->getKeyframe(out_frame->keyframeId_);

            //Notify callbacks
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

            if (addKeyFrameResult) { //add keyframe returns true if a loop closure was detected
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

    void OkvisSLAMSystem::PushIMU(const std::vector<ImuPair>& imu) {
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
        if (!sparse_map_vector.empty()) {
            sparse_map_vector.back()->lastKfTimestamp_ = sparse_map_vector.back()->lastKfTimestampDetect_ = 0.;
        }
        const auto newMap = std::make_shared<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>>();
        newMap->setEnableLoopClosure(parameters_.loopClosureParameters.enabled,strVocFile,true, new brisk::BruteForceMatcher());
        sparse_map_vector.push_back(newMap);
        // set it to the latest one
        active_map_index = static_cast<int>(sparse_map_vector.size())-1;
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

    void OkvisSLAMSystem::getTrajectory(std::vector<Eigen::Matrix4d>& trajOut){
        getActiveMap()->getTrajectory(trajOut);
    }

    std::shared_ptr<SparseMap<DBoW2::FBRISK::TDescriptor, DBoW2::FBRISK>> OkvisSLAMSystem:: getActiveMap() {
        if (0 <= active_map_index && active_map_index < sparse_map_vector.size()) {
            return sparse_map_vector[active_map_index];
        } else {
            std::cout << "Null map returned \n";
            return nullptr;
        }
    }
    
	void OkvisSLAMSystem::getMappedTrajectory(std::vector<int>& frameIdOut, std::vector<Eigen::Matrix4d>& trajOut) {
		sparseMap_.getMappedTrajectory(frameIdOut, trajOut);
	}


} //ark