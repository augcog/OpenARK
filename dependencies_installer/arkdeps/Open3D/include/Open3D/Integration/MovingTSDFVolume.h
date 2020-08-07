// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

//Created by Adam Chang
//Implementation of TSDF blocking with key frame association to allow loop closure on 3D Reconstructed segments in real time

#pragma once

#include <memory>
#include <map>
#include <unordered_map>
#include <mutex>
#include <chrono>

#include "Open3D/Integration/TSDFVolume.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Utility/Helper.h"

namespace open3d {
	namespace integration {

		class ScalableTSDFVolume;

		/// Class that implements a more memory efficient data structure for volumetric
		/// integration
		/// This implementation is based on the following repository:
		/// https://github.com/qianyizh/ElasticReconstruction/tree/master/Integrate
		/// The reference is:
		/// Q.-Y. Zhou and V. Koltun
		/// Dense Scene Reconstruction with Points of Interest
		/// In SIGGRAPH 2013
		///
		/// An observed depth pixel gives two types of information: (a) an approximation
		/// of the nearby surface, and (b) empty space from the camera to the surface.
		/// They induce two core concepts of volumetric integration: weighted average of
		/// a truncated signed distance function (TSDF), and carving. The weighted
		/// average of TSDF is great in addressing the Gaussian noise along surface
		/// normal and producing a smooth surface output. The carving is great in
		/// removing outlier structures like floating noise pixels and bumps along
		/// structure edges.

		class MovingTSDFVolume{
		public:
			MovingTSDFVolume(double voxel_length,
				double sdf_trunc,
				TSDFVolumeColorType color_type,
				double block_length);
			~MovingTSDFVolume();

		public:
			struct MeshUnit {
			public:
				MeshUnit() {}

			public:
				std::shared_ptr<geometry::TriangleMesh> mesh;
				Eigen::Matrix4d transform;
				int keyframe_num;
				Eigen::Vector3i block_loc;
			};

		public:
			void Reset();
			void Integrate(const geometry::RGBDImage &image,
				const camera::PinholeCameraIntrinsic &intrinsic,
				const Eigen::Matrix4d &extrinsic);
			std::shared_ptr<geometry::PointCloud> ExtractCurrentPointCloud();
			std::shared_ptr<geometry::TriangleMesh> ExtractCurrentTriangleMesh();
			std::shared_ptr<geometry::TriangleMesh> ExtractTotalTriangleMesh();
			std::shared_ptr<geometry::PointCloud> ExtractCurrentVoxelPointCloud();
			std::vector<std::pair<std::shared_ptr<geometry::TriangleMesh>, 
				Eigen::Matrix4d>> GetTriangleMeshes();
			void SetLatestKeyFrame(Eigen::Matrix4d transform, int key_frame_num);
			void UpdateKeyFrames(std::map<int, Eigen::Matrix4d> keyframes);
			std::vector<int> get_kf_ids();

		public:
			int volume_unit_resolution_;
			double volume_unit_length_;
			int depth_sampling_stride_;
			double block_length_;
			double sdf_trunc_;
			double voxel_length_;
			TSDFVolumeColorType color_type_;


		private:
			Eigen::Vector3i LocateBlock(const Eigen::Vector3d &point) {
				return Eigen::Vector3i((int)std::floor(point(0) / block_length_),
					(int)std::floor(point(1) / block_length_),
					(int)std::floor(point(2) / block_length_));
			}

			void MovingTSDFVolume::UpdateActiveVolume(Eigen::Matrix4d extrinsic);
			void MovingTSDFVolume::CombineMeshes(std::shared_ptr<geometry::TriangleMesh>& output_mesh, std::shared_ptr<geometry::TriangleMesh> mesh_to_combine);


			//stores triangles meshes of completed reconstructed blocks
			std::vector<std::shared_ptr<MeshUnit>> completed_meshes;


			//stores current scalable tsdf volume
			ScalableTSDFVolume * active_volume;
			Eigen::Matrix4d active_volume_transform;
			int active_volume_keyframe_num = -1;
			Eigen::Vector3i current_block;

			Eigen::Matrix4d latest_keyframe;
			int latest_keyframe_num;

			//temp for debug purposes
			int counter = 0;

			std::chrono::system_clock::time_point latest_loop_closure = std::chrono::system_clock::now();
			float time_threshold = 1.0;

		protected:
			std::mutex keyFrameLock;
			std::mutex meshLock;
		};

	}  // namespace integration
}  // namespace open3d
