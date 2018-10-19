#pragma once
#include "Detector.h"
#include "HumanBody.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>

namespace ark {
	/** Human detector class supports the detection and tracking human subjects in a RGB frame
	 * @see HumanDetector
	 */
	class HumanDetector : public Detector {
	public:
		HumanDetector(DetectionParams::Ptr params = nullptr);

	protected:
		void detect(cv::Mat & image) override;

	private:
		std::vector<HumanBody> humanBodies;

		cv::Rect lastHumanDetectionBox;

		cv::HOGDescriptor humanHOG;

		const int HUMAN_LOCAL_DETECT_RANGE = 50;

		const std::string MPII_PROTO_FILE_PATH = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_deploy_linevec_faster_4_stages.prototxt";

		const std::string MPII_WEIGHTS_FILE_PATH = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_iter_160000.caffemodel";

		void detectHumanHOG(const cv::Mat& frame);

		void detectBodyPose(const cv::Mat& frame);

		void detectHeadPose(const cv::Mat& frame);

		cv::Rect find_max_rec(const std::vector<cv::Rect>& found_filtered);
	};
}