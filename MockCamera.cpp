#include "MockCamera.h"

namespace ark {
	// Listing out all files in directory
	// https://www.boost.org/doc/libs/1_57_0/libs/filesystem/example/simple_ls.cpp
	MockCamera::MockCamera(std::string& file_path)
	{
		boost::filesystem::path image_dir(file_path);

		if (is_directory(image_dir)) {
			boost::filesystem::directory_iterator end_iter;
			for (boost::filesystem::directory_iterator dir_itr(image_dir); dir_itr != end_iter; ++dir_itr) {
				const auto& next_path = dir_itr->path().generic_string();
				file_names.emplace_back(next_path);
			}
			std::sort(file_names.begin(), file_names.end());
		}
	}

	// Reading from file OpenCV
	// https://docs.opencv.org/2.4/modules/core/doc/xml_yaml_persistence.html
	void MockCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
		cv::Mat & amp_map, cv::Mat & flag_map) {
		if (file_names.empty()) {
			return;
		}

		const auto path = file_names.front();
		file_names.pop_front();
		cv::FileStorage fs2(path, cv::FileStorage::READ);

		fs2["xyz_map"] >> xyz_map;

		fs2.release();
	}

	int MockCamera::getHeight() const {
		return 0;
	}

	int MockCamera::getWidth() const {
		return 0;
	}

	MockCamera::~MockCamera()
	{
		file_names.clear();
	}
}