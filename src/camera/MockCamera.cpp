#include "stdafx.h"
#include "openark/camera/MockCamera.h"

namespace ark {
	// Listing out all files in directory
	// https://www.boost.org/doc/libs/1_57_0/libs/filesystem/example/simple_ls.cpp
	MockCamera::MockCamera(const char* path)
	{
        typedef boost::filesystem::path fspath;
		fspath file_path(path);
		fspath intrin_path = file_path / "intrin.txt";
		fspath timestamps_path = file_path / "timestamp.txt";

        std::ifstream intrin_ifs(intrin_path.string());
        
		fspath depth_dir = file_path / "depth";
        if (intrin_ifs) {
            // depth files are depth maps (require intrinsics to convert)
            depth_dir = file_path / "depth_exr";
            std::string _garbage;
            intrin_ifs >> _garbage;
            intrin_ifs >> intr_fx;
            intrin_ifs >> _garbage;
            intrin_ifs >> intr_cx;
            intrin_ifs >> _garbage;
            intrin_ifs >> intr_fy;
            intrin_ifs >> _garbage;
            intrin_ifs >> intr_cy;
        }
        // else: assume depth files are xyz maps

        // read timestamps
        std::ifstream ts_ifs(timestamps_path.string());
        long long _tmp;
        while (ts_ifs >> _tmp) {
            timestamps.push_back(_tmp);
        }

		fspath rgb_dir = file_path / "rgb";
		fspath joint_dir = file_path / "joint";

		if (is_directory(depth_dir)) {
			boost::filesystem::directory_iterator end_iter;
			for (boost::filesystem::directory_iterator dir_itr(depth_dir); dir_itr != end_iter; ++dir_itr) {
				const auto& next_path = dir_itr->path().generic_string();
				depth_files.emplace_back(next_path);
			}
			std::sort(depth_files.begin(), depth_files.end());
		}

		if (is_directory(rgb_dir)) {
			boost::filesystem::directory_iterator end_iter;
			for (boost::filesystem::directory_iterator dir_itr(rgb_dir); dir_itr != end_iter; ++dir_itr) {
				const auto& next_path = dir_itr->path().generic_string();
				rgb_files.emplace_back(next_path);
			}
			std::sort(rgb_files.begin(), rgb_files.end());
		}

		if (is_directory(joint_dir)) {
			boost::filesystem::directory_iterator end_iter;
			for (boost::filesystem::directory_iterator dir_itr(joint_dir); dir_itr != end_iter; ++dir_itr) {
				const auto& next_path = dir_itr->path().generic_string();
				joint_files.emplace_back(next_path);
			}
			std::sort(joint_files.begin(), joint_files.end());
		}

		//ASSERT(depth_files.size() == rgb_files.size() && rgb_files.size() == joint_files.size());
	}

	// Reading from file OpenCV
	// https://docs.opencv.org/2.4/modules/core/doc/xml_yaml_persistence.html
	void MockCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map,
		cv::Mat & amp_map, cv::Mat & flag_map) {
		if (depth_files.empty() || rgb_files.empty() || joint_files.empty()) {
            std::cout << "MockCamera: No more files to read\n";
            badInputFlag = true;
			return;
		}

		const auto depth_path = depth_files.front();
		const auto rgb_path = rgb_files.front();
		const auto joint_path = joint_files.front();
		depth_files.pop_front();
		rgb_files.pop_front();
		joint_files.pop_front();
        if (timestamps.empty()) {
            timestamp = -1;
            deltaT = -1;
        }
        else {
            if (timestamp == -1) {
                deltaT = -1;
            }
            else {
                deltaT = timestamps.front() - timestamp;
            }
            timestamp = timestamps.front();
            timestamps.pop_front();
        }

		cv::Mat depth = cv::imread(depth_path, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
        if (intr_cy >= 0.) {
            // depth to xyz
            xyz_map = cv::Mat(depth.size(), CV_32FC3);
            float * inPtr; cv::Vec3f * outPtr;
            for (int r = 0; r < depth.rows; ++r) {
                inPtr = depth.ptr<float>(r);
                outPtr = xyz_map.ptr<cv::Vec3f>(r);
                for (int c = 0; c < depth.cols; ++c) {
                    const float z = inPtr[c];
                    outPtr[c] = cv::Vec3f(
                        (c - intr_cx) * z / intr_fx,
                        (r - intr_cy) * z / intr_fy,
                        z);
                }
            }
        }
        else {
            xyz_map = depth;
        }
		rgb_map = cv::imread(rgb_path);
		
		cv::FileStorage fs2(joint_path, cv::FileStorage::READ);
		fs2["joints"] >> joints;
		fs2.release();
	}
	
    long long MockCamera::getTimestamp() const
    {
        return timestamp;
    }

    long long MockCamera::getDeltaT() const
    {
        return deltaT;
    }

	std::vector<cv::Point>& MockCamera::getJoints() {
		return joints;
	}

	std::vector<cv::Point> MockCamera::getJoints() const {
		return joints;
	}

	int MockCamera::getHeight() const {
		return 0;
	}

	int MockCamera::getWidth() const {
		return 0;
	}

    bool MockCamera::hasRGBMap() const
    {
        return true;
    }

	bool MockCamera::hasNext() const {
		return depth_files.size() != 0;
	}

	MockCamera::~MockCamera()
	{
		depth_files.clear();
		rgb_files.clear();
		joint_files.clear();
	}
}
