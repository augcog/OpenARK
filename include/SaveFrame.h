#ifndef OPENARK_SAVEFRAME_H
#define OPENARK_SAVEFRAME_H

#include <mutex>
#include <thread>
#include <map>
#include <string>

#include <opencv2/opencv.hpp>


namespace ark{

    class SaveFrame{
    public:
        SaveFrame(std::string folderPath);

        //void OnKeyFrameAvailable(const RGBDFrame &keyFrame);

        //void OnFrameAvailable(const RGBDFrame &frame);

		void SaveFrame::frameWrite(cv::Mat imRGB, cv::Mat depth, Eigen::Matrix4d traj, int frameId);

        //ark::RGBDFrame frameLoad(int frameId);

    private:

        //Main Loop thread
        std::string folderPath;
        std::string rgbPath;
        std::string depthPath;
        std::string tcwPath;
        std::string depth_to_tcw_Path;

		int frame_counter;

    };
}

#endif  //#define OPENARK_SAVEFRAME_H