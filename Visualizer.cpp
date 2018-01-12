#include "stdafx.h"
#include "version.h"
#include "Visualizer.h"
#include "Util.h"

namespace ark {
    pcl::visualization::PCLVisualizer * Visualizer::viewer = nullptr;

    /***
    Maps matrix values to [0, 255] for viewing
    ***/
    cv::Mat Visualizer::visualizeMatrix(cv::Mat &input)
    {
        cv::Mat img;
        cv::normalize(input, img, 0, 255, cv::NORM_MINMAX, CV_8UC3);
        return img;
    }

    /***
    RGB depth map visualization
    ***/
    cv::Mat Visualizer::visualizeDepthMap(cv::Mat &depthMap)
    {
        cv::Mat img;
        cv::normalize(depthMap, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(img, img, cv::COLORMAP_HOT);
        return img;
    }

    cv::Mat Visualizer::visualizeXYZMap(cv::Mat &xyzMap)
    {
        cv::Mat channels[3];
        cv::split(xyzMap, channels);
        return visualizeDepthMap(channels[2]);
    }

    cv::Mat Visualizer::visualizeHand(cv::Mat xyzMap, const Hand hand)
    {
        cv::Mat displayImg;

        if (xyzMap.type() == CV_32FC3)
        {
            displayImg = Visualizer::visualizeXYZMap(xyzMap);
        }
        else
        {
            displayImg = xyzMap;
        }

        float unitWid = (float)xyzMap.cols / 640;

        cv::circle(displayImg, hand.center_ij, (int)(unitWid * 10), cv::Scalar(255, 0, 0));

        Point2i box = Point2i(unitWid * 7, unitWid * 7);
        cv::rectangle(displayImg, hand.wrist_ij[0] - box, hand.wrist_ij[0] + box,
            cv::Scalar(0, 255, 255), unitWid * 2.0);
        cv::rectangle(displayImg, hand.wrist_ij[1] - box, hand.wrist_ij[1] + box,
            cv::Scalar(0, 255, 255), unitWid * 2.0);

        cv::circle(displayImg, hand.center_ij, hand.circle_radius,
            cv::Scalar(100, 100, 100), 1);

        for (int i = 0; i < std::min(hand.fingers_ij.size(), hand.defects_ij.size()); ++i)
        {
            cv::circle(displayImg, hand.fingers_ij[i], (int)(unitWid * 6), cv::Scalar(0, 0, 255), -1);
            cv::line(displayImg, hand.defects_ij[i], hand.fingers_ij[i], cv::Scalar(0, 0, 255), (int)(unitWid * 2));

            std::stringstream sstr;
            sstr << setprecision(1) << std::fixed << util::euclideanDistance(hand.defects_xyz[i], hand.fingers_xyz[i]) * 100;
            cv::putText(displayImg,
                sstr.str(),
                (hand.defects_ij[i] + hand.fingers_ij[i]) / 2 - Point2i(15, 0), 0, 0.7 * unitWid, cv::Scalar(0, 255, 255), 1);

            cv::circle(displayImg, hand.defects_ij[i], (int)(unitWid * 5), cv::Scalar(255, 0, 200), -1);
            cv::line(displayImg, hand.defects_ij[i], hand.center_ij, cv::Scalar(255, 0, 200), (int)(unitWid * 2));

            sstr.str("");
            sstr << util::euclideanDistance(hand.defects_xyz[i], hand.center_xyz) * 100;

            cv::putText(displayImg,
                sstr.str(),
                (hand.defects_ij[i] + hand.center_ij) / 2 - Point2i(15, 0), 0, 0.4 * unitWid, cv::Scalar(0, 255, 255), 1);



            cv::putText(displayImg,
                std::to_string(hand.fingers_ij.size() - i),
                hand.fingers_ij[i] + Point2i(-6, -18), 0, 0.7 * unitWid, cv::Scalar(255, 0, 255), 1);
        }

        return displayImg;
    }

    void Visualizer::visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        initPCLViewer();
        viewer->setBackgroundColor(0, 0, 0);

        if (!viewer->updatePointCloud(cloud))
            viewer->addPointCloud(cloud);

        viewer->spinOnce();
    }

    cv::Mat Visualizer::visualizePlaneRegression(cv::Mat &input_mat, std::vector<double> &equation, const double threshold, bool clicked)
    {
        cv::Mat output_mat;

        if (input_mat.type() == CV_32FC3)
        {
            output_mat = Visualizer::visualizeXYZMap(input_mat);
        }

        else
        {
            output_mat = input_mat;
        }

        if (equation.size() < 3)
        {
            return output_mat;
        }

        auto rowSize = input_mat.rows;
        auto colSize = input_mat.cols;
        cv::Scalar color;

        if (clicked)
        {
            color = cv::Scalar(255, 255, 0);
        }

        else
        {
            color = cv::Scalar(0, 255, 0);
        }

        auto pointsDetected = 0;

        for (auto r = 0; r < rowSize; r++)
        {

            for (auto c = 0; c < colSize; c++)
            {

                double x = input_mat.at<Point3f>(r, c)[0];
                double y = input_mat.at<Point3f>(r, c)[1];
                double z = input_mat.at<Point3f>(r, c)[2];

                if (z == 0)
                {
                    continue;
                }

                auto z_hat = equation[0] * x + equation[1] * y + equation[2];
                auto r_squared = (z - z_hat) * (z - z_hat);

                if (r_squared < threshold)
                {
                    cv::circle(output_mat, Point2i(c, r), 1, color, -1);
                    pointsDetected++;
                }

            }

        }
        return output_mat;
    }

    void Visualizer::visualizePlanePoints(cv::Mat &input_mat, std::vector<Point2i> indicies)
    {
        for (auto i = 0; i < indicies.size(); i++) {
            input_mat.at<uchar>(indicies[i].y, indicies[i].x) = static_cast<uchar>(255);
        }
    }

    void Visualizer::visulizePolygonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        if (cloud.get()->width == 0)
        {
            return;
        }
        initPCLViewer();

        // Normal estimation*
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);
        //* normals should not contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(0.025);

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18); // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        viewer->setBackgroundColor(0, 0, 0);

        if (!viewer->updatePolygonMesh(triangles))
            viewer->addPolygonMesh(triangles);

        viewer->spinOnce();
    }

    bool Visualizer::initPCLViewer() {
        if (viewer != nullptr) return false;
        viewer = new pcl::visualization::PCLVisualizer("Point Cloud");
        return viewer != nullptr;
    }
}