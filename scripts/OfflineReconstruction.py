"""
Reconstructs output data from SaveFrame.cpp using Open3D's Integration
"""

import open3d as o3d 
import numpy as np
import os, sys
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def main():

	if (len(sys.argv) != 2):
		print("usage: python " + sys.argv[0] + " <folder containing frames (RGB, depth, tcw)>")
		exit()

	config = dict()

	config.update({"tsdf_cubic_size":0.5})
	config.update({"max_depth": 4.00})
	config.update({"voxel_size": 0.1})
	config.update({"max_depth_diff": 5.00})
	config.update({"tsdf_cubic_size": 3.0})
	config.update({"python_multi_threading": True})
    

	rgb_images = sys.argv[1] + "/RGB/"
	depth_images = sys.argv[1] + "/depth/"
	transforms = sys.argv[1] + "/tcw/"

	if (not os.path.isdir(rgb_images) or not os.path.isdir(depth_images) or not os.path.isdir(transforms)):
		print("Cannot find frames")
		print("Check directories: ", rgb_images, " ", depth_images, " ", transforms)
		exit()

	cam_intr = np.zeros((3,3))
	cam_intr[0][0] = 612.081
	cam_intr[1][1] = 612.307
	cam_intr[0][2] = 318.254
	cam_intr[1][2] = 237.246
	cam_intr[2][2] = 1.0


	intr = o3d.camera.PinholeCameraIntrinsic()
	intr.set_intrinsics(640, 480, cam_intr[0][0], cam_intr[1][1], cam_intr[0][2], cam_intr[1][2])

	print("===================================")
	print("===================================")
	print("Reconstructing using this intrinsic matrix:")
	print(intr.intrinsic_matrix)
	print("===================================")
	print("===================================")
	for key in config.keys():
		print(key, ": ", config[key])
	print("===================================")
	print("===================================")

	volume = o3d.integration.ScalableTSDFVolume(
		voxel_length= config["voxel_size"],
		sdf_trunc=config["voxel_size"] * 5,
		color_type=o3d.integration.TSDFVolumeColorType.RGB8)

	lst = []

	for frame in os.listdir(rgb_images):
		frame_id = int(frame[:frame.find(".")])
		lst.append(frame_id)

	lst = sorted(lst)

	max_frameid = lst[-1]

	for frame_id in lst:	

		print("processing frame id: ", frame_id, "/", max_frameid, end = '\r')

		frame_id = str(frame_id)

		color_raw = o3d.io.read_image(rgb_images + frame_id + ".jpg")
		depth_raw = o3d.io.read_image(depth_images + frame_id + ".png")

		rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_raw, depth_raw, depth_trunc=config["max_depth"], convert_rgb_to_intensity=False)
		
		cam_pose = np.zeros((4,4))

		cam_pose = np.loadtxt(transforms + frame_id + ".txt")

		cam_pose = np.linalg.inv(cam_pose)

		volume.integrate(rgbd_image, intr, cam_pose)


	print("Finished Integrating")
	mesh = volume.extract_triangle_mesh()

	mesh.compute_vertex_normals()

	o3d.io.write_triangle_mesh("mesh.ply", mesh)

	print("Saved mesh to mesh.ply")



if __name__ == "__main__":
	main()