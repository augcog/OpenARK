#pragma once
// C++ Libraries
#include <filesystem>
#include <boost/filesystem.hpp>
#include <iostream>

// OpenCV Libraries
#include "Version.h"
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

// OpenARK Libraries
#include "DepthCamera.h"

namespace ark {
	/**
	 * This class defines the behavior of a camera that reads from a data file rather than a live camera
	**/
	class MockCamera : public DepthCamera
	{
	public:
		explicit MockCamera(const char* path);

		int getHeight() const override;

		int getWidth() const override;

		void update();

		cv::Mat& getXYZMap();
		
		cv::Mat& getRGBMap();

		std::vector<cv::Point>& getJoints();

		~MockCamera();

		bool hasNext() const;

	protected:
		/**
		* Gets the new frame from the sensor (implements functionality).
		* Updates xyzMap and ir_map.
		*/
		void update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
			cv::Mat & amp_map, cv::Mat & flag_map) override;

	private:
		int height;
		int width;
		std::deque<std::string> depth_files;
		std::deque<std::string> rgb_files;
		std::deque<std::string> joint_files;
		std::vector<cv::Point> joints;

        // camera intrinsics, if available
        double intr_fx, intr_fy, intr_cx, intr_cy = -1.;
	};
}
