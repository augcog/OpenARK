#ifndef OPENARK_SAVEFRAME_H
#define OPENARK_SAVEFRAME_H

#include <mutex>
#include <thread>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>
#include "openark/util/Types.h"


namespace ark{

    class SaveFrame{
    public:
        SaveFrame(std::string folderPath);
        //void OnKeyFrameAvailable(const RGBDFrame &keyFrame);
        //void OnFrameAvailable(const RGBDFrame &frame);

        void frameWrite(const cv::Mat& imRGB, const cv::Mat& depth, const Eigen::Matrix4d& traj, int frameId); 
        void frameWriteMapped(const cv::Mat& imRGB, const cv::Mat& depth, const Eigen::Matrix4d& traj, int frameId, int mapId); 
        void updateTransforms(std::map<int, Eigen::Matrix4d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4d>>> &keyframemap); 
        void writeActiveFrames(std::vector<int> frame_ids);  
        
        ark::RGBDFrame frameLoad(int frameId);

    private:
        //Main Loop thread
        std::string folderPath;
        std::string rgbPath;
        std::string depthPath;
        std::string tcwPath;
        std::string mapIdLog;
        std::string activeFramesLog;
        std::string depth_to_tcw_Path;
        std::vector<int> frame_ids;

    };
}

#endif  //#define OPENARK_SAVEFRAME_H
