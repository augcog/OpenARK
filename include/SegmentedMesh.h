#pragma once

#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"
#include "Open3D/geometry/PointCloud.h"
#include "Open3D/geometry/TriangleMesh.h"
#include "Open3D/camera/PinholeCameraIntrinsic.h"
#include "Types.h"
#include <map>
#include <set>
#include <unordered_map>
#include <mutex>
#include <chrono>

namespace ark {

	class SegmentedMesh {

	public:
		SegmentedMesh(double voxel_length,
			double sdf_trunc,
			open3d::integration::TSDFVolumeColorType color_type,
			double block_length, bool blocking = true);

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
		void DeleteMapsAfterIndex(int map_index);
		void Render(std::vector<std::vector<Eigen::Vector3d>> &mesh_vertices, std::vector<std::vector<Eigen::Vector3d>> &mesh_colors, 
		std::vector<std::vector<Eigen::Vector3i>> &mesh_triangles, std::vector<Eigen::Matrix4d> &mesh_transforms, std::vector<int> &mesh_enabled);
		void WriteMeshes();

	public:
		double block_length_;
		double sdf_trunc_;
		double voxel_length_;
		open3d::integration::TSDFVolumeColorType color_type_;
		bool blocking_;


	private:
		Eigen::Vector3i LocateBlock(const Eigen::Vector3d &point) {
			return Eigen::Vector3i((int)std::floor(point(0) / block_length_),
				(int)std::floor(point(1) / block_length_),
				(int)std::floor(point(2) / block_length_));
		}

		void UpdateActiveVolume(Eigen::Matrix4d extrinsic);
		void CombineMeshes(std::shared_ptr<open3d::geometry::TriangleMesh>& output_mesh, std::shared_ptr<open3d::geometry::TriangleMesh> mesh_to_combine);


		//stores triangles meshes of completed reconstructed blocks
		std::vector<std::shared_ptr<MeshUnit>> completed_meshes;


		//stores current scalable tsdf volume
		open3d::integration::ScalableTSDFVolume * active_volume;
		MapKeyFrame::Ptr active_volume_keyframe;
		int active_volume_map_index = 0;

		Eigen::Vector3i current_block;

		MapKeyFrame::Ptr latest_keyframe;
		int active_map_index = 0;
		int archive_index = -1;

		bool start_new_block = false;

		//temp for debug purposes
		int counter = 0;

		std::chrono::system_clock::time_point latest_loop_closure = std::chrono::system_clock::now();
		float time_threshold = 1.0;

	protected:
		std::mutex keyFrameLock;
		std::mutex meshLock;
	};
}