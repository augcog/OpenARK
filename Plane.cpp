#include "Plane.h"

Plane::Plane()
{

}

Plane::Plane(cv::Mat &src)
{
	// Initialize Variables
	cloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
	normals = new pcl::PointCloud <pcl::Normal>::Ptr(new pcl::PointCloud <pcl::Normal>);
	down_cloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	down_normals = new pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	upsampled_colored_cloud = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	initializeCloud(src);

	if (cloud->get()->width < CLOUD_SIZE_THRESHOLD) {
		return;
	}

	compute();
}

Plane::~Plane() {
}

int Plane::compute()
{
	voxelDownsample();

	// calculate normals
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree
		= boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	calculateNormals(normal_estimator, tree);

	// region grow with normals
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	regionGrow(reg, tree);

	// find index of dominant plane
	int index_max = -1;
	for (int i = 0; i < clusters.size(); i++) {
		if (index_max == -1 || clusters[i].indices.size() > clusters[index_max].indices.size()) {
			index_max = i;
		}
	}
	if (index_max == -1) {
		return -1;
	}

	computePlaneLSE(clusters[index_max]);
	computeSphereLSE(clusters[index_max]);

	num_plane_points = computePlaneIndices();
	num_sphere_points = computeSphereIndices();

	return 0;
}

void Plane::computePlaneLSE(pcl::PointIndices &ind)
{
	Eigen::MatrixXd A(ind.indices.size(), 3);
	Eigen::MatrixXd b(ind.indices.size(), 1);

	for (int i = 0; i < ind.indices.size(); i++) {
		int index = ind.indices[i];
		A(i, 0) = down_cloud->get()->points[index].x;
		A(i, 1) = down_cloud->get()->points[index].y;
		A(i, 2) = 1;
		b(i, 0) = down_cloud->get()->points[index].z;
	}

	Eigen::MatrixXd mul = A.transpose() * A;
	Eigen::MatrixXd inv = mul.inverse();
	Eigen::MatrixXd b_hat = A.transpose() * b;
	Eigen::MatrixXd vals = inv * b_hat;

	plane_equation.clear();
	plane_equation.resize(3);
	plane_equation[0] = vals(0, 0);
	plane_equation[1] = vals(1, 0);
	plane_equation[2] = vals(2, 0);
}

void Plane::computeSphereLSE( pcl::PointIndices &ind)
{
	Eigen::MatrixXd A(ind.indices.size(), 4);
	Eigen::MatrixXd b(ind.indices.size(), 1);

	for (int i = 0; i < ind.indices.size(); i++) {
		int index = ind.indices[i];
		A(i, 0) = 1;
		A(i, 1) = down_cloud->get()->points[index].x;
		A(i, 2) = down_cloud->get()->points[index].y;
		A(i, 3) = down_cloud->get()->points[index].z;
		b(i, 0) = down_cloud->get()->points[index].x * down_cloud->get()->points[index].x +
			down_cloud->get()->points[index].y * down_cloud->get()->points[index].y +
			down_cloud->get()->points[index].z * down_cloud->get()->points[index].z;
	}

	Eigen::MatrixXd mul = A.transpose() * A;
	Eigen::MatrixXd inv = mul.inverse();
	Eigen::MatrixXd b_hat = A.transpose() * b;
	Eigen::MatrixXd vals = inv * b_hat;

	double a0 = vals(0, 0);
	double coef_a = vals(1, 0) / 2;
	double coef_b = vals(2, 0) / 2;
	double coef_c = vals(3, 0) / 2;
	double radius = sqrt(a0 + coef_a * coef_a + coef_b * coef_b + coef_c * coef_c);

	sphere_equation.clear();
	sphere_equation.resize(4);
	sphere_equation[0] = coef_a;
	sphere_equation[1] = coef_b;
	sphere_equation[2] = coef_c;
	sphere_equation[3] = radius;
}

int Plane::computePlaneIndices()
{
	int rowSize = depth_img.rows;
	int colSize = depth_img.cols;
	plane_indices.clear();
	int num_points_detected = 0;
	for (int r = 0; r < rowSize; r++) {
		for (int c = 0; c < colSize; c++) {
			double x = depth_img.at<cv::Vec3f>(r, c)[0];
			double y = depth_img.at<cv::Vec3f>(r, c)[1];
			double z = depth_img.at<cv::Vec3f>(r, c)[2];

			if (z > 0) {
				double z_hat = plane_equation[0] * x + plane_equation[1] * y + plane_equation[2];
				double r_squared = (z - z_hat) * (z - z_hat);

				if (r_squared < R_SQUARED_DISTANCE_THRESHOLD) {
					plane_indices.push_back(cv::Point2i(c, r));
					num_points_detected++;
				}
			}
		}
	}
	return num_points_detected;
}

int Plane::computeSphereIndices()
{
	int rowSize = depth_img.rows;
	int colSize = depth_img.cols;
	sphere_indices.clear();
	int pointsDetected = 0;
	for (int r = 0; r < rowSize; r++) {
		for (int c = 0; c < colSize; c++) {
			double x = depth_img.at<cv::Vec3f>(r, c)[0];
			double y = depth_img.at<cv::Vec3f>(r, c)[1];
			double z = depth_img.at<cv::Vec3f>(r, c)[2];

			if (z > 0) {
				double radius = sqrt((x - sphere_equation[0]) * (x - sphere_equation[0])
					+ (y - sphere_equation[1]) * (y - sphere_equation[1])
					+ (z - sphere_equation[2]) * (z - sphere_equation[2]));
				double radial_r_squared = (radius - sphere_equation[3]) * (radius - sphere_equation[3]);

				if (radial_r_squared < R_SQUARED_DISTANCE_THRESHOLD) {
					sphere_indices.push_back(cv::Point2i(c, r));
					pointsDetected++;
				}
			}
		}
	}
	return pointsDetected;
}

void Plane::regionGrow(pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, pcl::search::Search<pcl::PointXYZ>::Ptr tree)
{
	if (down_cloud->get()->width < 100 && down_normals->get()->width < 100) {
		return;
	}

	reg.setMinClusterSize(20);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(*down_cloud);
	reg.setInputNormals(*down_normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(2.5);
	reg.extract(clusters);
}

void Plane::voxelDownsample()
{
	pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*cloud->get(), *temp_cloud);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(temp_cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);
	pcl::fromPCLPointCloud2(*cloud_filtered, *down_cloud->get());
}

void Plane::calculateNormals(pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator,
	pcl::search::Search<pcl::PointXYZ>::Ptr tree)
{
	normal_estimator.setNumberOfThreads(NUM_CORES);
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(*down_cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*down_normals->get());
}

void Plane::initializeCloud(cv::Mat &src)
{
	depth_img = src.clone();
	int rowSize = src.rows;
	int colSize = src.cols;
	int cloudSize = 0;
	for (size_t r = 0; r < rowSize; r++) {
		for (size_t c = 0; c < colSize; c++) {
			if (src.at<cv::Vec3f>(r, c)[2] != 0) {
				cloudSize++;
			}
		}
	}
	cloud->get()->width = cloudSize;
	cloud->get()->height = 1;
	cloud->get()->points.resize(cloud->get()->width * cloud->get()->height);

	int index = 0;
	for (size_t r = 0; r < rowSize; r++) {
		for (size_t c = 0; c < colSize; c++) {
			if (src.at<cv::Vec3f>(r, c)[2] != 0) {
				cloud->get()->points[index].x = src.at<cv::Vec3f>(r, c)[0];
				cloud->get()->points[index].y = src.at<cv::Vec3f>(r, c)[1];
				cloud->get()->points[index].z = src.at<cv::Vec3f>(r, c)[2];
				index++;
			}
		}
	}

}

pcl::PointCloud<pcl::PointXYZ>::Ptr Plane::getCloud()
{
	return *cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Plane::getDownCloud()
{
	return *down_cloud;
}

std::vector<double> Plane::getPlaneEquation()
{
	return plane_equation;
}

std::vector<double> Plane::getSphereEquation()
{
	return sphere_equation;
}

std::vector<cv::Point2i> Plane::getPlaneIndicies()
{
	return plane_indices;
}

std::vector<cv::Point2i> Plane::getSphereIndices()
{
	return sphere_indices;
}