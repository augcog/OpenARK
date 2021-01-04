#ifndef OPENARK_SAVEFRAME_H
#define OPENARK_SAVEFRAME_H

#include <mutex>
#include <thread>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>
#include "Types.h"


namespace ark{

    class SaveFrame{
    public:
        SaveFrame(std::string folderPath);

        //void OnKeyFrameAvailable(const RGBDFrame &keyFrame);

        //void OnFrameAvailable(const RGBDFrame &frame);

		void SaveFrame::frameWrite(cv::Mat imRGB, cv::Mat depth, Eigen::Matrix4d traj, int frameId);
        void SaveFrame::frameWriteMapped(cv::Mat imRGB, cv::Mat depth, Eigen::Matrix4d traj, int frameId, int mapId);
		void SaveFrame::updateTransforms(std::map<int, Eigen::Matrix4d> keyframemap);
        void SaveFrame::writeActiveFrames(std::vector<int> frame_ids);

        ark::RGBDFrame SaveFrame::frameLoad(int frameId);

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