#pragma once
#include "hand_and_avatar/Detector.h"
#include "hand_and_avatar/avatar/HumanBody.h"
#include "hand_and_avatar/avatar/Avatar.h"

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

		void detectPoseRGB(cv::Mat & rgbMap);

		double update(cv::Mat & xyzMap, cv::Mat & rgbMap, std::vector<cv::Point>& rgbJoints, double deltat = -1.0);

		std::vector<std::shared_ptr<HumanBody>>& getHumanBodies();

		std::shared_ptr<HumanAvatar> getAvatarModel();

        /** Data file paths */
        static const std::string MPII_PROTO_FILE_PATH;
        static const std::string MPII_WEIGHTS_FILE_PATH;
        static const std::string FACE_LBFMODEL_FILE_PATH;
        static const std::string FACE_HAARCASCADE_FILE_PATH;
        static const std::string HUMAN_MODEL_PATH;

        /** Names of shapekeys to use */
        static const std::vector<std::string> HUMAN_MODEL_SHAPE_KEYS;

        // utils

        /** OpenPose MPI model output joint indices  */
        enum OpenPoseMPIJoint {
            HEAD, NECK, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST, LEFT_SHOULDER,
            LEFT_ELBOW, LEFT_WRIST, RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE,
            LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, CHEST, BACKGROUND, _COUNT
        };

        /** convert 2D MPI-15 joints to 3D SMPL joints, given xyzMap and joints, and outputs 3D joints into out;
          * if a joint's position cannot be determined, the x component of the output position will be NAN.
          * @param complete if true, tries to guess all SMPL joints even ones with no correspondences using predetermined linear combinations of nearby joints;
          *        else returns only matched joints
          * @see toMPIJoints
          */
		static void toSMPLJoints(const cv::Mat & xyzMap, const std::vector<cv::Point> & mpi_joints,
			HumanAvatar::EigenCloud_T & out, bool complete = true);

        /** conver SMPL joints to 2d MPI-15 joints.
          * if a joint's position cannot be determined, the x component of the output position will be NAN.
          * Templated to allow use with Ceres autodiff
          * @see toSMPLJoints
          */
        template<class T, class Cloud_T>
		static void toMPIJoints(const cv::Vec4d & intrin, const Cloud_T & smpl_joints,
            Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor> & out) {
            out = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>(OpenPoseMPIJoint::_COUNT - 1, 2);
            typedef HumanAvatar::JointType smpl_j;
            typedef OpenPoseMPIJoint mpi_j;

            //out.col(0).setConstant(T(NAN));
            out.row(mpi_j::LEFT_HIP) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_HIP));
            out.row(mpi_j::RIGHT_HIP) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_HIP));
            out.row(mpi_j::LEFT_KNEE) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_KNEE));
            out.row(mpi_j::RIGHT_KNEE) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_KNEE));
            out.row(mpi_j::LEFT_ANKLE) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_ANKLE));
            out.row(mpi_j::RIGHT_ANKLE) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_ANKLE));
            out.row(mpi_j::LEFT_ELBOW) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_ELBOW));
            out.row(mpi_j::RIGHT_ELBOW) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_ELBOW));
            out.row(mpi_j::LEFT_WRIST) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_WRIST));
            out.row(mpi_j::RIGHT_WRIST) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_WRIST));
            out.row(mpi_j::LEFT_SHOULDER) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::L_SHOULDER));
            out.row(mpi_j::RIGHT_SHOULDER) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::R_SHOULDER));
            out.row(mpi_j::NECK) = _projectToImage<T>(intrin, smpl_joints.row(smpl_j::NECK));

            // below we infer joints
            out.row(mpi_j::CHEST) = _projectToImage<T>(intrin,
                    (smpl_joints.row(smpl_j::SPINE2) + smpl_joints.row(smpl_j::SPINE1)) * 0.5);
        }

        static inline Eigen::Vector2d projectToImage(const cv::Vec4d & intrin, const Eigen::Vector3d & pt) {
            return _projectToImage<double>(intrin, pt.transpose()).transpose();
        }

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

        /* helper to project from world to image space */
        template<class T>
        static inline Eigen::Matrix<T, 1, 2, Eigen::RowMajor>
                      _projectToImage(const cv::Vec4d & intrin, const Eigen::Matrix<T, 1, 3, Eigen::RowMajor> & pt) {
            Eigen::Matrix<T, 1, 2, Eigen::RowMajor> out;
            out[0] = -pt[0] * intrin[0] / pt[2] + intrin[1];
            out[1] = pt[1] * intrin[2] / pt[2] + intrin[3];
            return out;
        }
	};
}
