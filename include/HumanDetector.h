#pragma once
#include "Detector.h"
#include "HumanBody.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>

namespace ark {
	struct KeyPoint {
		KeyPoint(cv::Point point, float probability) {
			this->id = -1;
			this->point = point;
			this->probability = probability;
		}

		int id;
		cv::Point point;
		float probability;
	};

	struct ValidPair {
		ValidPair(int aId, int bId, float score) {
			this->aId = aId;
			this->bId = bId;
			this->score = score;
		}

		int aId;
		int bId;
		float score;
	};

	const std::string keypointsMapping[] = {
		"Nose", "Neck",
		"R-Sho", "R-Elb", "R-Wr",
		"L-Sho", "L-Elb", "L-Wr",
		"R-Hip", "R-Knee", "R-Ank",
		"L-Hip", "L-Knee", "L-Ank",
		"R-Eye", "L-Eye", "R-Ear", "L-Ear"
	};
	/** Human detector class supports the detection and tracking human subjects in a RGB frame
	 * @see HumanDetector
	 */
	class HumanDetector : public Detector {
	public:
		HumanDetector(DetectionParams::Ptr params = nullptr);

		std::vector<std::shared_ptr<HumanBody>> getHumanBodies();

	protected:
		void detect(cv::Mat & image) override;

	private:
		std::vector<std::shared_ptr<HumanBody>> human_bodies;

		/** Human Detection Variables **/
		cv::Rect lastHumanDetectionBox;

		cv::HOGDescriptor humanHOG;

		const int HUMAN_LOCAL_DETECT_RANGE = 50;

		/** Body Pose Variables **/
		cv::dnn::Net openPoseNet;

		const double POSE_CONFIDENCE_THRESHOLD = 0.3;

		const std::string MPII_PROTO_FILE_PATH = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_deploy_linevec_faster_4_stages.prototxt";

		const std::string MPII_WEIGHTS_FILE_PATH = "C:\\dev\\openpose\\models\\pose\\mpi\\pose_iter_160000.caffemodel";

		/** Head Pose Variables **/
		cv::CascadeClassifier faceDetector;

		cv::Ptr<cv::face::Facemark> facemark;

		std::vector<cv::Point3d> face_3D_model_points;

		void detectHumanHOG(const cv::Mat& frame);

		void detectBodyPose(const cv::Mat& frame);

		void detectHeadPose(const cv::Mat& frame);

		cv::Rect find_max_rec(const std::vector<cv::Rect>& found_filtered);

		void splitNetOutputBlobToParts(cv::Mat& netOutputBlob, const cv::Size& targetSize, std::vector<cv::Mat>& netOutputParts);

		void getKeyPoints(cv::Mat& probMap, double threshold, std::vector<KeyPoint>& keyPoints);

		void populateColorPalette(std::vector<cv::Scalar>& colors, int nColors);

		void populateInterpPoints(const cv::Point& a, const cv::Point& b, int numPoints, std::vector<cv::Point>& interpCoords);

		void getValidPairs(const std::vector<cv::Mat>& netOutputParts,
			const std::vector<std::vector<KeyPoint>>& detectedKeypoints,
			std::vector<std::vector<ValidPair>>& validPairs,
			std::set<int>& invalidPairs);

		void getPersonwiseKeypoints(const std::vector<std::vector<ValidPair>>& validPairs,
			const std::set<int>& invalidPairs,
			std::vector<std::vector<int>>& personwiseKeypoints);
	};
}