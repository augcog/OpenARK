#include "stdafx.h"
#include "Version.h"
#include "util/Visualizer.h"
#include "util/Util.h"

namespace ark {
    pcl::visualization::PCLVisualizer::Ptr Visualizer::viewer = nullptr;

    /***
    Maps matrix values to [0, 255] for viewing
    ***/
    void Visualizer::visualizeMatrix(const cv::Mat & input, cv::Mat & output)
    {
        if (output.rows == 0) output.create(input.size(), CV_8UC3);
        cv::normalize(input, output, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    }

    /***
    RGB depth map visualization
    ***/
   
    void Visualizer::visualizeXYZMap(const cv::Mat & xyzMap, cv::Mat & output, float max_depth)
    {
        cv::Mat depth;
        cv::extractChannel(xyzMap, depth, 2);
        output = depth * 255 / max_depth;
        output.convertTo(output, CV_8UC1);
        cv::applyColorMap(output, output, cv::COLORMAP_HOT);
    }

    void Visualizer::visualizeNormalMap(const cv::Mat & normal_map, cv::Mat & output, 
                                        int resolution)
    {
        if (output.rows == 0) output.create(normal_map.size() / resolution, CV_8UC3);

        for (int i = 0; i < output.rows; ++i) {
            const Vec3f * inPtr = normal_map.ptr<Vec3f>(i * resolution);
            Vec3b * outPtr = output.ptr<Vec3b>(i);

            for (int j = 0; j < output.cols; ++j) {
                int jj = j * resolution;

                if (inPtr[jj] == Vec3f(0, 0, 0)) {
                    outPtr[j] = Vec3b(0, 0, 0);
                }
                else {
                    outPtr[j][2] = (uchar)((inPtr[jj][0] + 1.0) * 127.5);
                    outPtr[j][1] = (uchar)((inPtr[jj][1] + 1.0) * 127.5);
                    outPtr[j][0] = (uchar)(-inPtr[jj][2] * 127.5 + 127.5);
                }
            }
        }

        cv::resize(output, output, normal_map.size(), (double)resolution,
            (double)resolution, cv::INTER_LINEAR);
    }

    void Visualizer::visualizeHand(const cv::Mat & background, cv::Mat & output,
                Hand * hand, double display,
                const std::vector<std::shared_ptr<FramePlane> > * touch_planes)
    {
        if (background.type() == CV_32FC3)
        {
            Visualizer::visualizeXYZMap(background, output);
        }
        else if (output.rows == 0)
        {
            output = background.clone();
        }

        float unitWid = std::min((float)background.cols / 640, 1.75f);

        // draw contour
        if (hand->getContour().size() > 2) {
            cv::polylines(output, hand->getContour(), true, cv::Scalar(0, 255, 0), 1);
        }

        Point2i center = hand->getPalmCenterIJ();
        Vec3f centerXYZ = hand->getPalmCenter();

        // draw circle at center of hand
        cv::circle(output, center, std::round(unitWid * 10), cv::Scalar(255, 0, 0));

        // draw wrist
        Point2i box = Point2i(unitWid * 6, unitWid * 6);
        const std::vector<Point2i> & wrists = hand->getWristIJ();
        cv::rectangle(output, wrists[0] - box, wrists[0] + box,
            cv::Scalar(255, 255, 0), unitWid * 1.5);
        cv::rectangle(output, wrists[1] - box, wrists[1] + box,
            cv::Scalar(255, 255, 0), unitWid * 1.5);

        // faint outline of largest inscribed circle
        cv::circle(output, center, hand->getCircleRadius(), cv::Scalar(100, 100, 100), 
            std::round(unitWid));

        const std::vector<Point2i> & fingers = hand->getFingersIJ();
        const std::vector<Point2i> & defects = hand->getDefectsIJ();
        const std::vector<Vec3f> & fingersXYZ = hand->getFingers();
        const std::vector<Vec3f> & defectsXYZ = hand->getDefects();

        // detect touches
        if (touch_planes != nullptr) {
            std::vector<std::pair<int, std::vector<int> > > fingerTouching;
            cv::Scalar touchColor(200, 255, 0);
            cv::Size touchSz(roundf(22 * unitWid), roundf(22 * unitWid));
            Point2i offset = (Point2i)(cv::Point2f(-11, -11) * unitWid);
            int wid = roundf(3 * unitWid);

            if (hand->touchingPlanes(*touch_planes, fingerTouching)) {
                for (auto p : fingerTouching) {
                    cv::rectangle(output, 
                        cv::Rect(fingers[p.first] + offset, touchSz), touchColor, wid);

                    for (int i : p.second) {
                        (*touch_planes)[i]->getCenterIJ();
                    }
                }
            }
        }

        for (int i = 0; i < hand->getNumFingers(); ++i)
        {
            // draw fingers
            cv::line(output, defects[i], fingers[i], cv::Scalar(0, 150, 255), roundf(unitWid * 2));
            cv::circle(output, fingers[i], roundf(unitWid * 7), cv::Scalar(0, 0, 255), -1);

            std::stringstream sstr;
            // defect-fingertip distances
            sstr << setprecision(1) << std::fixed <<
                util::euclideanDistance(defectsXYZ[i], fingersXYZ[i]) * 100;

            cv::putText(output,
                sstr.str(), (defects[i] + fingers[i]) / 2 - Point2i(15, 0), 0,
                0.7 * unitWid, cv::Scalar(0, 255, 255), 1);

            cv::circle(output, defects[i], roundf(unitWid * 5), cv::Scalar(255, 0, 200), -1);
            cv::line(output, defects[i], center, cv::Scalar(255, 0, 200), roundf(unitWid * 2));

            // defect-center distances
            sstr.str("");
            sstr << util::euclideanDistance(defectsXYZ[i], centerXYZ) * 100;
            cv::putText(output,
                sstr.str(),
                (defects[i] + center) / 2 - Point2i(15, 0), 0,
                0.4 * unitWid, cv::Scalar(0, 255, 255), 1);
        }
        
        // draw dominant direction arrow
        Point2f dir = hand->getDominantDirection();
        cv::arrowedLine(output, Point2f(center), Point2f(center) + dir * 50,
            cv::Scalar(200, 200, 120), std::round(unitWid * 3), 8, 0, 0.3);
            
        if (display < FLT_MAX) {
            // draw provided display text
            std::stringstream sstr;
            sstr << std::setprecision(3) << std::fixed << display;
            Point2i dispPt = center - Point2i((int)sstr.str().size() * 8, 0);
            cv::putText(output, sstr.str(), dispPt, 0, 0.8, cv::Scalar(255, 255, 255), 1);
        }
    }

    void Visualizer::visualizePlaneRegression(const cv::Mat & input_mat, cv::Mat & output, std::vector<double> &equation, const double threshold, bool clicked)
    {
        if (input_mat.type() == CV_32FC3)
        {
            Visualizer::visualizeXYZMap(input_mat, output);
        }

        else
        {
            output = input_mat;
        }

        if (equation.size() < 3) return;

        cv::Scalar color = cv::Scalar(255 * clicked, 255, 0);
        int pointsDetected = 0;

        for (int r = 0; r < input_mat.rows; r++)
        {
            const Vec3f * ptr = input_mat.ptr<Vec3f>(r);

            for (int c = 0; c < input_mat.cols; c++)
            {
                const Vec3f & v = ptr[c];

                if (v[2] == 0) continue;

                double r_squared =
                    util::pointPlaneDistance((Vec3d)v, equation[0], equation[1], equation[2]);

                if (r_squared < threshold)
                {
                    cv::circle(output, Point2i(c, r), 1, color, -1);
                    ++ pointsDetected;
                }

            }

        }
    }

    void Visualizer::visualizePlanePoints(cv::Mat &input_mat, std::vector<Point2i> indicies)
    {
        for (auto i = 0; i < indicies.size(); i++) {
            input_mat.at<uchar>(indicies[i].y, indicies[i].x) = static_cast<uchar>(255);
        }
    }

    int Visualizer::createPCLViewport(double xmin, double ymin, double xmax, double ymax)
    {
        initPCLViewer();
        int id;
        viewer->createViewPort(xmin, ymin, xmax, ymax, id);
        return id;
    }

    pcl::visualization::PCLVisualizer::Ptr Visualizer::getPCLVisualizer()
    {
        initPCLViewer();
        return viewer;
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
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewport");
        viewer->registerKeyboardCallback([](const pcl::visualization::KeyboardEvent & evt) {
            // add handler to allow quit
            unsigned char k = evt.getKeyCode();
            if (k == 'Q' || k == 'q' || k == 27) {
                std::exit(0);
            }
        });
        return viewer != nullptr;
    }

	void Visualizer::visualizeFaceLandmarks(cv::Mat &im, std::vector<cv::Point2f> &landmarks) {
		// Draw face for the 68-point model.
		if (landmarks.size() == 68) {
			//drawPolyline(im, landmarks, 0, 16);           // Jaw line
			//drawPolyline(im, landmarks, 17, 21);          // Left eyebrow
			//drawPolyline(im, landmarks, 22, 26);          // Right eyebrow
			//drawPolyline(im, landmarks, 27, 30);          // Nose bridge
			//drawPolyline(im, landmarks, 30, 35, true);    // Lower nose
			//drawPolyline(im, landmarks, 36, 41, true);    // Left eye
			//drawPolyline(im, landmarks, 42, 47, true);    // Right Eye
			//drawPolyline(im, landmarks, 48, 59, true);    // Outer lip
			//drawPolyline(im, landmarks, 60, 67, true);    // Inner lip
		} else { 
			for (int i = 0; i < landmarks.size(); i++) {
				circle(im, landmarks[i], 3, cv::Scalar(255, 200, 0), cv::FILLED);
			}
		}
	}

	/*void Visualizer::drawPolyline(cv::Mat &im, const std::vector<cv::Point2f> &landmarks, const int start, const int end, bool isClosed) {
		std::vector<cv::Point> points;

		for (int i = start; i <= end; i++) {
			points.push_back(cv::Point(landmarks[i].x, landmarks[i].y));
		}

		cv::polylines(im, points, isClosed, cv::Scalar(255, 200, 0), 2, 16);
	}*/
}
