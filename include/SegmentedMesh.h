#pragma once

#include "CameraSetup.h"
#include "OkvisSLAMSystem.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"
#include "Open3D/geometry/PointCloud.h"
#include "Open3D/geometry/TriangleMesh.h"
#include "Open3D/camera/PinholeCameraIntrinsic.h"
#include "Types.h"
#include "SaveFrame.h"
#include <map>
#include <set>
#include <unordered_map>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

namespace ark {

	std::shared_ptr<open3d::geometry::RGBDImage> generateRGBDImageFromCV(cv::Mat color_mat, cv::Mat depth_mat, double max_depth);

	class SegmentedMesh {

	public:
		SegmentedMesh(std::string& recon_config, OkvisSLAMSystem& slam, CameraSetup* camera, bool blocking = true);
		SegmentedMesh(std::string& recon_config);
		SegmentedMesh();

		//~SegmentedMesh();

	public:
		struct MeshUnit {
		public:
			MeshUnit() {}

		public:
			std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
			MapKeyFrame::Ptr keyframe;
			int mesh_map_index;
			Eigen::Vector3i block_loc;
		};

	public:
		void Reset();
		void Integrate(const open3d::geometry::RGBDImage &image,
			const open3d::camera::PinholeCameraIntrinsic &intrinsic,
			const Eigen::Matrix4d &extrinsic);
		std::shared_ptr<open3d::geometry::PointCloud> ExtractCurrentPointCloud();
		std::shared_ptr<open3d::geometry::TriangleMesh> ExtractCurrentTriangleMesh();
		std::shared_ptr<open3d::geometry::TriangleMesh> ExtractTotalTriangleMesh();
		std::shared_ptr<open3d::geometry::PointCloud> ExtractCurrentVoxelPointCloud();
		std::vector<std::pair<std::shared_ptr<open3d::geometry::TriangleMesh>, 
			Eigen::Matrix4d>> GetTriangleMeshes();
		void SetLatestKeyFrame(MapKeyFrame::Ptr frame);
		std::vector<int> get_kf_ids();
		void StartNewBlock();
		void SetActiveMapIndex(int map_index);

		std::tuple<std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>>, 
			std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>>,
			std::shared_ptr<std::vector<std::vector<Eigen::Vector3i>>>, 
			std::shared_ptr<std::vector<Eigen::Matrix4d>>, std::shared_ptr<std::vector<int>>> GetOutputVectors();

		void AddRenderMutex(std::mutex* render_mutex, std::string render_mutex_key);
		void RemoveRenderMutex(std::string render_mutex_key);
		void WriteMeshes();

		void SetIntegrationEnabled(bool enabled);

	public:
		open3d::integration::TSDFVolumeColorType color_type_ = open3d::integration::TSDFVolumeColorType::RGB8;
		int integration_frame_stride_ = 3;
		int extraction_frame_stride_ = 60;

		int camera_height_, camera_width_;

	private:

		//initialize
		void Initialize(std::string& recon_config, bool blocking);

		//setup sets up all callbacks
		void Setup(OkvisSLAMSystem& slam, CameraSetup* camera);

		Eigen::Vector3i LocateBlock(const Eigen::Vector3d &point) {
			return Eigen::Vector3i((int)std::floor(point(0) / block_length_),
				(int)std::floor(point(1) / block_length_),
				(int)std::floor(point(2) / block_length_));
		}

		void readConfig(std::string& recon_config);
		void UpdateActiveVolume(Eigen::Matrix4d extrinsic);
		void CombineMeshes(std::shared_ptr<open3d::geometry::TriangleMesh>& output_mesh, std::shared_ptr<open3d::geometry::TriangleMesh> mesh_to_combine);
		void UpdateOutputVectors();


		//stores triangles meshes of completed reconstructed blocks
		std::vector<std::shared_ptr<MeshUnit>> completed_meshes;


		//stores current scalable tsdf volume
		open3d::integration::ScalableTSDFVolume * active_volume;
		MapKeyFrame::Ptr active_volume_keyframe;
		int active_volume_map_index = 0;

		Eigen::Vector3i current_block;

		MapKeyFrame::Ptr latest_keyframe;
		int active_map_index = 0;

		bool start_new_block = false;

		int frame_counter_ = 1;

		std::chrono::system_clock::time_point latest_loop_closure = std::chrono::system_clock::now();
		float time_threshold = 1.0;


		double block_length_;
		double sdf_trunc_;
		double voxel_length_;
		bool blocking_;
		double max_depth_;
		bool do_integration_;

		std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>> mesh_vertices;
		std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>> mesh_colors;
		std::shared_ptr<std::vector<std::vector<Eigen::Vector3i>>> mesh_triangles;
		std::shared_ptr<std::vector<Eigen::Matrix4d>> mesh_transforms;
		std::shared_ptr<std::vector<int>> mesh_enabled;

		std::unordered_map<std::string, std::mutex *> render_mutexes;

	protected:
		std::mutex keyFrameLock;
		std::mutex meshLock;
	};
}