#include "stdafx.h"

// OpenARK Libraries
#include "Version.h"
#ifdef PMDSDK_ENABLED
#include "PMDCamera.h"
#endif
#ifdef RSSDK_ENABLED
#include "SR300Camera.h"
#endif
#ifdef RSSDK2_ENABLED
#include "RS2Camera.h"
#include "MockCamera.h"
#endif

#include "Core.h"
#include "Visualizer.h"
#include "Avatar.h"

using namespace ark;

void filter_by_depth(cv::Mat& xyz_map, double min_depth, double max_depth) {
    for (int r = 0; r < xyz_map.rows; ++r)
    {
        Vec3f * ptr = xyz_map.ptr<Vec3f>(r);

        for (int c = 0; c < xyz_map.cols; ++c)
        {
            if (ptr[c][2] > max_depth || ptr[c][2] < min_depth) {
                ptr[c][0] = ptr[c][1] = ptr[c][2] = 0.0f;
            }
        }
    }
}

void filter_by_plane(cv::Mat& xyz_map) {

}

int main(int argc, char ** argv) {
    google::InitGoogleLogging(argv[0]);

    printf("Welcome to OpenARK v %s Demo\n\n", VERSION);
    // seed the rng
    srand(time(NULL));

    const std::string IMG_PATH = "D:\\Programming\\3VR\\OpenARK_dataset\\human-basic2-D435\\capture_06.yml";
    // gender-neutral model
    const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_neutral_lbs_10_207_0_v1.0.0/";
    // male model
    //const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_m_lbs_10_207_0_v1.0.0/";
    // female model
    //const std::string HUMAN_MODEL_PATH = "D:/DataSets/human/SMPL/models/basicModel_f_lbs_10_207_0_v1.0.0/";

    const std::vector<std::string> SHAPE_KEYS = { "shape000.pcd", "shape001.pcd",
                                           "shape002.pcd", "shape003.pcd", "shape004.pcd",
                                           "shape005.pcd", "shape006.pcd", "shape007.pcd",
                                           "shape008.pcd", "shape009.pcd"
    };

    // initialize parameters
    DetectionParams::Ptr params = DetectionParams::create();

    PlaneDetector::Ptr planeDetector = std::make_shared<PlaneDetector>();
    cv::Mat xyzMap;
    cv::FileStorage fs2(IMG_PATH, cv::FileStorage::READ);
    fs2["xyz_map"] >> xyzMap;
    fs2.release();
    cv::Mat floodFillMap = xyzMap.clone();

    filter_by_depth(floodFillMap, 1, 3);

    planeDetector->update(xyzMap);
    const std::vector<FramePlane::Ptr> & planes = planeDetector->getPlanes();
    if (planes.size()) {
        for (FramePlane::Ptr plane : planes) {
            util::removePlane<Vec3f>(floodFillMap, floodFillMap, plane->equation, 0.0015);
        }

        std::vector<Point2i> allIndices;
        allIndices.reserve(xyzMap.cols * xyzMap.rows);

        cv::Mat out(xyzMap.rows, xyzMap.cols, CV_32FC3);
        //cv::Mat out = floodFillMap.clone();
        int numPts = util::floodFill(floodFillMap, Point2i(410, 200), 2.0f,
            &allIndices, nullptr, &out,
            1, 0, 200.0f);

        cv::circle(floodFillMap, Point(410, 200), 2, cv::Scalar(1, 1, 1), 2);
        //cv::imshow("Mid", floodFillMap);
        cv::imshow("Fill", out);
        cv::imshow("Depth Map", xyzMap);

        auto humanCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
        util::toPointCloud(out, humanCloud, true, true);
        auto viewer = Visualizer::getPCLVisualizer();

        HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS);
        //ava.setCenterPosition(util::cloudCenter(humanCloud));
        ava.update();

        const std::string MODEL_CLOUD_NAME = "model_cloud";
        const std::string DATA_CLOUD_NAME = "human_cloud";
        viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME, 0);
        viewer->addPointCloud(humanCloud, DATA_CLOUD_NAME, 0);
        ava.fit(humanCloud);
        ava.visualize(viewer, "ava_");
        viewer->removePointCloud(MODEL_CLOUD_NAME);
        viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME, 0);

        viewer->spinOnce();
    }

    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
