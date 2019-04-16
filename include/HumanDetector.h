#pragma once
#include "Detector.h"
#include "HumanBody.h"
#include "Avatar.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

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

		double update(cv::Mat & xyzMap, cv::Mat & rgbMap, std::vector<cv::Point>& rgbJoints);

		std::vector<std::shared_ptr<HumanBody>>& getHumanBodies();

		std::shared_ptr<HumanAvatar> getAvatarModel();

	protected:
		void detect(cv::Mat & image) override;

	private:
		std::shared_ptr<HumanAvatar> ava;

		std::vector<std::shared_ptr<HumanBody>> human_bodies;

		bool begin_tracking;

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

		void segmentAvatar(const cv::Mat & xyz_map, const std::vector<cv::Point2i> & points_on_target,
			cv::Mat & out);

		int filterByHeight(cv::Mat& xyz_map, int feet);

		void toSMPLJoints(const cv::Mat & xyzMap, const std::vector<cv::Point> & mpi_joints,
			HumanAvatar::EigenCloud_T & out);

		template<class T>
		boost::shared_ptr<pcl::PointCloud<T>> denoisePointCloud(boost::shared_ptr<pcl::PointCloud<T>> & human_cloud,
			float resolution = 0.05f, int mean_k = 50, double std_dev_mul = 1.0) {
			auto humanCloud_filtered = boost::make_shared<pcl::PointCloud<T>>();
			auto humanCloud_down = boost::make_shared<pcl::PointCloud<T>>();

			pcl::StatisticalOutlierRemoval<T> sor;
			sor.setInputCloud(human_cloud);
			sor.setMeanK(mean_k);
			sor.setStddevMulThresh(std_dev_mul);
			sor.filter(*humanCloud_filtered);

			pcl::VoxelGrid<T> voxel_processor;
			voxel_processor.setInputCloud(humanCloud_filtered);
			voxel_processor.setLeafSize(resolution, resolution, resolution);
			voxel_processor.filter(*humanCloud_down);
			return humanCloud_down;
		}
	};
}