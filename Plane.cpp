#include "stdafx.h"
#include "version.h"

#include "Plane.h"
#include "Visualizer.h"

namespace ark {
    Plane::Plane()
    {

    }

    Plane::Plane(const cv::Mat &src, const std::vector<Point2i> & pts, int num_pts)
    {
        depth_img = &src;
        this->points = &pts;
        this->num_points = num_pts;

        if (num_pts < 0 || num_pts > pts.size())
            num_pts = (int)pts.size();

        if (num_pts < CLOUD_SIZE_THRESHOLD) return;

        initializeCloud();

        compute();
        //findPlaneRANSAC();
    }

    Plane::~Plane()
    {
    }

    int Plane::findPlaneRANSAC()
    {
        // referencing: http://pointclouds.org/documentation/tutorials/planar_segmentation.php

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(*cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            num_plane_points = 0;
            return -1;
        }

        plane_equation.resize(4);

        for (int i = 0; i < 4; ++i) {
            plane_equation[i] = coefficients->values[i];
        }

        num_plane_points = computePlaneIndices();

        return 0;
    }

    int Plane::compute()
    {
        voxelDownsample();

        // calculate normals
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
        auto ptr = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        auto tree = boost::static_pointer_cast<pcl::search::Search<pcl::PointXYZ>>(ptr);
        calculateNormals(normal_estimator, tree);

        // region grow with normals
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        regionGrow(reg, tree);

        // find index of dominant plane
        int index_max = 0;
        for (int i = 1; i < clusters.size(); ++i)
        {
            if (clusters[i].indices.size() > clusters[index_max].indices.size())
                index_max = i;
        }

        if (clusters.size() == 0) return -1;

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

        for (auto i = 0; i < ind.indices.size(); i++)
        {
            auto index = ind.indices[i];
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

    void Plane::computeSphereLSE(pcl::PointIndices &ind)
    {
        Eigen::MatrixXd A(ind.indices.size(), 4);
        Eigen::MatrixXd b(ind.indices.size(), 1);

        for (auto i = 0; i < ind.indices.size(); i++)
        {
            auto index = ind.indices[i];
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

        auto a0 = vals(0, 0);
        auto coef_a = vals(1, 0) / 2;
        auto coef_b = vals(2, 0) / 2;
        auto coef_c = vals(3, 0) / 2;
        auto radius = sqrt(a0 + coef_a * coef_a + coef_b * coef_b + coef_c * coef_c);

        sphere_equation.clear();
        sphere_equation.resize(4);
        sphere_equation[0] = coef_a;
        sphere_equation[1] = coef_b;
        sphere_equation[2] = coef_c;
        sphere_equation[3] = radius;
    }

    int Plane::computePlaneIndices()
    {
        plane_indices.clear();
        int num_points_detected = 0;

        for (int i = 0; i < num_points; ++i)
        {
            const Point2i * pt = &(*points)[i];

            const Point3f vec = depth_img->at<Point3f>(*pt);
            double x = vec[0], y = vec[1], z = vec[2];
            if (plane_equation.size() == 4) {
                double a = plane_equation[0], b = plane_equation[1], c = plane_equation[2], d = plane_equation[3];

                if (z > 0)
                {
                    double dist =
                        abs(a * x + b * y + c * z + d) /
                        sqrt(a*a + b*b + c*c);

                    if (dist < R_SQUARED_DISTANCE_THRESHOLD)
                    {
                        plane_indices.push_back(Point2i(pt->x, pt->y));
                        ++num_points_detected;
                    }
                }
            }
            else {
                if (z > 0)
                {
                    double a = plane_equation[0], b = plane_equation[1], c = plane_equation[2];
                    double z_hat = abs(a * x + b * y + c);
                    double r_sq = (z - z_hat) *(z - z_hat);

                    if (r_sq < 0.0001)
                    {
                        plane_indices.push_back(Point2i(pt->x, pt->y));
                        ++num_points_detected;
                    }
                }

            }
        }

        if (num_points_detected < num_points * 0.9) {
            num_points_detected = 0;
            plane_indices.clear();
        }

        return num_points_detected;
    }

    int Plane::computeSphereIndices()
    {
        int rowSize = depth_img->rows;
        int colSize = depth_img->cols;
        sphere_indices.clear();
        int pointsDetected = 0;
        for (int r = 0; r < rowSize; r++)
        {
            for (int c = 0; c < colSize; c++)
            {
                double x = depth_img->at<Point3f>(r, c)[0];
                double y = depth_img->at<Point3f>(r, c)[1];
                double z = depth_img->at<Point3f>(r, c)[2];

                if (z > 0)
                {
                    double radius = sqrt((x - sphere_equation[0]) * (x - sphere_equation[0])
                        + (y - sphere_equation[1]) * (y - sphere_equation[1])
                        + (z - sphere_equation[2]) * (z - sphere_equation[2]));
                    double radial_r_squared = (radius - sphere_equation[3]) * (radius - sphere_equation[3]);

                    if (radial_r_squared < R_SQUARED_DISTANCE_THRESHOLD)
                    {
                        sphere_indices.push_back(Point2i(c, r));
                        pointsDetected++;
                    }
                }
            }
        }
        return pointsDetected;
    }

    int Plane::drawSphereRegressionPoints(cv::Mat& output_mat, cv::Mat& input_mat, std::vector<double>& equation, const int rowSize, const int colSize, const double threshold, bool clicked)
    {
        return 0;
    }

    void Plane::regionGrow(pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> &reg, pcl::search::Search<pcl::PointXYZ>::Ptr tree)
    {
        if (down_cloud->get()->width < 100 && down_normals->get()->width < 100)
        {
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

    void Plane::voxelDownsample() const
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
        pcl::search::Search<pcl::PointXYZ>::Ptr tree) const
    {
        normal_estimator.setNumberOfThreads(NUM_CORES);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(*down_cloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*down_normals->get());
    }

    void Plane::initializeCloud()
    {
        // Initialize Variables
        this->cloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
        this->normals = new pcl::PointCloud <pcl::Normal>::Ptr(new pcl::PointCloud <pcl::Normal>);
        this->down_cloud = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        this->down_normals = new pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
        this->upsampled_colored_cloud = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        this->cloud->get()->width = num_points;
        this->cloud->get()->height = 1;

        auto & cloudPoints = this->cloud->get()->points;
        cloudPoints.resize(num_points);

        for (int i = 0; i < num_points; ++i)
        {
            const Point3f vec = depth_img->at<Point3f>((*points)[i]);
            cloudPoints[i].x = vec[0];
            cloudPoints[i].y = vec[1];
            cloudPoints[i].z = vec[2];
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Plane::getCloud() const
    {
        return *cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Plane::getDownCloud() const
    {
        return *down_cloud;
    }

    std::vector<double> Plane::getPlaneEquation() const
    {
        return plane_equation;
    }

    std::vector<double> Plane::getSphereEquation() const
    {
        return sphere_equation;
    }

    std::vector<Point2i> Plane::getPlaneIndicies() const
    {
        return plane_indices;
    }

    std::vector<Point2i> Plane::getSphereIndices() const
    {
        return sphere_indices;
    }
}
