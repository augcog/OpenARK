#include "stdafx.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/dnn.hpp>
#include <opencv2/face.hpp>
#include <opencv2/ximgproc.hpp>

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

static void filterByDepth(cv::Mat& xyz_map, double min_depth, double max_depth) {
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

static void segmentAvatar(const cv::Mat & xyz_map, const std::vector<cv::Point2i> & points_on_target,
    cv::Mat & out) {
    cv::Mat floodFillMap = xyz_map.clone();
    filterByDepth(floodFillMap, 1, 3);

    // initialize plane detector
    DetectionParams::Ptr params = DetectionParams::create();
    params->normalResolution = 3;
    params->planeFloodFillThreshold = 0.13;
    params->handPlaneMinNorm = 0.01;
    PlaneDetector::Ptr planeDetector = std::make_shared<PlaneDetector>();
    planeDetector->setParams(params);
    planeDetector->update(xyz_map);

    //cv::Mat normalVis;
    //Visualizer::visualizeNormalMap(planeDetector->getNormalMap(), normalVis, params->normalResolution);
    //cv::imshow("Normals", normalVis);

    const std::vector<FramePlane::Ptr> & planes = planeDetector->getPlanes();
    std::cerr << planes.size() << " Planes Found\n";
    for (FramePlane::Ptr plane : planes) {
        util::removePlane<Vec3f>(floodFillMap, floodFillMap, plane->equation, 0.004);
    }

    cv::imshow("PlaneRemove", floodFillMap);

    std::vector<Point2i> allIndices;
    allIndices.reserve(xyz_map.cols * xyz_map.rows);

    out = cv::Mat(xyz_map.rows, xyz_map.cols, CV_32FC3);

    cv::Mat color(xyz_map.size(), CV_8U);
    color = cv::Scalar(255);

    for (const auto & point : points_on_target) {
        if (point.x < 0 || color.at<unsigned char>(point) != 255) continue;
        util::floodFill(floodFillMap, point, 0.06f,
            &allIndices, nullptr, &out,
            1, 0, 0.0f, &color);
    }
}

static void face_direction()
{
    cv::CascadeClassifier faceDetector("C:\\opencv-3.4.3\\data\\haarcascades\\haarcascade_frontalface_alt2.xml");

    // Create an instance of Facemark
    cv::Ptr<cv::face::Facemark> facemark = cv::face::FacemarkLBF::create();

    // Load landmark detector
    facemark->loadModel("C:\\proj\\openark\\lbfmodel.yaml");

    cv::VideoCapture cam(0);
    cv::Mat frame, gray;

    while (cam.read(frame)) {
        // Find face
        std::vector<cv::Rect> faces;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect faces
        faceDetector.detectMultiScale(gray, faces);

        // There can be more than one face in the image. Hence, we 
        // use a vector of vector of points. 
        std::vector<std::vector<cv::Point2f>> landmarks;

        // Run landmark detector
        bool success = facemark->fit(frame, faces, landmarks);

        std::vector<cv::Point2d> image_points;
        if (success && landmarks[0].size() == 68) {
            Visualizer::visualizeFaceLandmarks(frame, landmarks[0]);

            image_points.push_back(landmarks[0][30]);    // Nose tip
            image_points.push_back(landmarks[0][8]);     // Chin
            image_points.push_back(landmarks[0][36]);    // Left eye left corner
            image_points.push_back(landmarks[0][45]);    // Right eye right corner
            image_points.push_back(landmarks[0][60]);    // Left Mouth corner
            image_points.push_back(landmarks[0][64]);    // Right mouth corner
        }
        else {
            cv::imshow("Facial Landmark Detection", frame);
            continue;
        }

        // 3D model points.

        std::vector<cv::Point3d> model_points;

        model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
        model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
        model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
        model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
        model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
        model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner

                                                                             // Camera internals
        double focal_length = frame.cols; // Approximate focal length.
        Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

                                                                                // Output rotation and translation
        cv::Mat rotation_vector; // Rotation in axis-angle form
        cv::Mat translation_vector;

        // Solve for pose
        cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

        std::vector<cv::Point3d> nose_end_point3D;
        std::vector<cv::Point2d> nose_end_point2D;
        nose_end_point3D.push_back(cv::Point3d(0, 0, 1000.0));

        cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

        for (int i = 0; i < image_points.size(); i++) {
            circle(frame, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
        }

        cv::line(frame, image_points[0], nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);

        cv::imshow("Facial Landmark Detection", frame);
    }
}

template<class T>
static boost::shared_ptr<pcl::PointCloud<T>> denoisePointCloud(boost::shared_ptr<pcl::PointCloud<T>> & human_cloud,
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

/** OpenPose MPI model output joint indices */
enum OpenPoseMPIJoint {
    HEAD, NECK, RIGHT_SHOULDER, RIGHT_ELBOW, RIGHT_WRIST, LEFT_SHOULDER,
    LEFT_ELBOW, LEFT_WRIST, RIGHT_HIP, RIGHT_KNEE, RIGHT_ANKLE,
    LEFT_HIP, LEFT_KNEE, LEFT_ANKLE, CHEST, BACKGROUND, _COUNT
};

static void estimatePoseRGB(const cv::Mat & frame, const cv::Mat & xyzMap, std::vector<cv::Point> & points, float thresh = 0.5) {
    /*
    static const int POSE_PAIRS[14][2] =
    {
        { 0,1 },{ 1,2 },{ 2,3 },
        { 3,4 },{ 1,5 },{ 5,6 },
        { 6,7 },{ 1,14 },{ 14,8 },{ 8,9 },
        { 9,10 },{ 14,11 },{ 11,12 },{ 12,13 }
    };*/

    const int nPoints = 15; // ignore 'background' point

    static const std::string protoFile = "C:\\proj\\openark\\openpose_models\\pose\\mpi\\pose_deploy_linevec_faster_4_stages.prototxt";
    static const std::string weightsFile = "C:\\proj\\openark\\openpose_models\\pose\\mpi\\pose_iter_160000.caffemodel";

    cv::dnn::Net net = cv::dnn::readNetFromCaffe(protoFile, weightsFile);

    //cv::Mat frame = cv::imread("D:\\Programming\\3VR\\openpose_models\\womanwalking.jpg"); // DEBUG
    cv::Mat frameCopy = frame.clone();

    //cv::ximgproc::dtFilter(frame, frame, frame, 1.0, 1.0);

    //cv::imshow("Human", frame);

    // Prepare the frame to be fed to the network
    cv::Mat inpBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, frame.size(), cv::Scalar(0, 0, 0));

    // Set the prepared object as the input blob of the network
    net.setInput(inpBlob);

    cv::Mat output = net.forward();
    int H = output.size[2], W = output.size[3];

    // find the position of the body parts
    points.resize(nPoints);
    //cv::Mat mask(xyzMap.size(), CV_8UC1);

    cv::Mat confMap = frame.clone();

    for (int n = 0; n < nPoints; n++) {
        // Probability map of corresponding body's part.
        cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));

        cv::Point2f p(-1, -1);
        cv::Point maxLoc;
        double prob;

        cv::minMaxLoc(probMap, 0, &prob, 0, &maxLoc);
        cv::Mat probMap_big; cv::resize(probMap, probMap_big, frame.size());
        for (int i = 0; i < confMap.rows; ++i) {
            float * pm_ptr = probMap_big.ptr<float>(i);
            Vec3b * vis_ptr = confMap.ptr<cv::Vec3b>(i);
            for (int j = 0; j < confMap.cols; ++j) {
                if (pm_ptr[j] > thresh) {
                    vis_ptr[j][2] = int(vis_ptr[j][2] * (1.0 - pm_ptr[j])) + int(255.0 * pm_ptr[j]);
                }
            }
        }
        if (prob > thresh) {
            p = maxLoc;
            p.x *= (float)frame.cols / W;
            p.y *= (float)frame.rows / H;

            //cv::circle(frameCopy, cv::Point((int)p.x, (int)p.y), 8, cv::Scalar(0, 255, 255), -1);
            //cv::putText(frameCopy, cv::format("%d", n), cv::Point((int)p.x, (int)p.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
        points[n] = p;
    }

    cv::imshow("Confidence", confMap);
    /*
    int nPairs = sizeof(POSE_PAIRS) / sizeof(POSE_PAIRS[0]);

    for (int n = 0; n < nPairs; n++) {

        // lookup 2 connected body/hand parts

        cv::Point2f partA = points[POSE_PAIRS[n][0]];
        cv::Point2f partB = points[POSE_PAIRS[n][1]];

        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
            continue;


        cv::line(frame, partA, partB, cv::Scalar(0, 255, 255), 8);
        cv::circle(frame, partA, 8, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, partB, 8, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Output-Keypoints", frameCopy);
    cv::imshow("Output-Skeleton", frame);
    cv::waitKey(0);
    */
}

static void toSMPLJoints(const cv::Mat & xyzMap, const std::vector<cv::Point> & mpi_joints,
                         HumanAvatar::EigenCloud_T & out) {
    typedef HumanAvatar::EigenCloud_T cloud;
    typedef HumanAvatar::JointType smpl_j;
    typedef OpenPoseMPIJoint mpi_j;
    
    cloud mpi(mpi_joints.size(), 3);

    for (int i = 0; i < mpi.rows(); ++i) {
        auto r = mpi.row(i);
        if (mpi_joints[i].x == -1) {
            r[0] = -1e12;
        } else {
            Vec3f joint_3d = util::averageAroundPoint(xyzMap, mpi_joints[i], 12);
            r[0] = joint_3d[0];
            r[1] = -joint_3d[1];
            r[2] = -joint_3d[2];
        }
    }

    // 'forward'-facing direction for avatar
    Eigen::Matrix<double, 1, 3> up = mpi.row(mpi_j::NECK) - mpi.row(mpi_j::CHEST);
    up.normalize();
    auto forward = up.cross(mpi.row(mpi_j::RIGHT_HIP) - mpi.row(mpi_j::LEFT_HIP));
    forward.normalize();
 
    double unit = (mpi.row(mpi_j::NECK) - mpi.row(mpi_j::CHEST)).norm() * 0.4;

    out = cloud((int)smpl_j::_COUNT, 3);
    out.row(smpl_j::PELVIS) = mpi.row(mpi_j::LEFT_HIP) * 0.5 + mpi.row(mpi_j::RIGHT_HIP) * 0.5 - forward * unit * 0.42;
    out.row(smpl_j::R_HIP) = mpi.row(mpi_j::LEFT_HIP) * 0.8 + mpi.row(mpi_j::LEFT_KNEE) * 0.2 - forward * unit * 0.3;
    out.row(smpl_j::L_HIP) = mpi.row(mpi_j::RIGHT_HIP) * 0.8 + mpi.row(mpi_j::RIGHT_KNEE) * 0.2 - forward * unit * 0.3;
    out.row(smpl_j::R_KNEE) = mpi.row(mpi_j::LEFT_KNEE);
    out.row(smpl_j::L_KNEE) = mpi.row(mpi_j::RIGHT_KNEE);
    out.row(smpl_j::R_ANKLE) = mpi.row(mpi_j::LEFT_ANKLE);
    out.row(smpl_j::L_ANKLE) = mpi.row(mpi_j::RIGHT_ANKLE);
    out.row(smpl_j::SPINE1) = mpi.row(mpi_j::CHEST) * 0.4 + mpi.row(mpi_j::LEFT_HIP) * 0.3 + mpi.row(mpi_j::RIGHT_HIP) * 0.3 - forward * unit * 0.6;
    out.row(smpl_j::SPINE2) = mpi.row(mpi_j::CHEST) - forward * unit * 0.65;
    out.row(smpl_j::SPINE3) = mpi.row(mpi_j::CHEST) * 0.8 + mpi.row(mpi_j::LEFT_SHOULDER) * 0.1 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.1 - forward * unit * 0.35;
    out.row(smpl_j::HEAD) = mpi.row(mpi_j::NECK) * 0.8 + mpi.row(mpi_j::HEAD) * 0.2 - up * unit * 0.2;
    out.row(smpl_j::NECK) = mpi.row(mpi_j::NECK) * 0.3 + mpi.row(mpi_j::LEFT_SHOULDER) * 0.35 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.35;
    out.row(smpl_j::R_SHOULDER) = mpi.row(mpi_j::LEFT_SHOULDER) - up * unit * 0.25 + forward * unit * 0.1;
    out.row(smpl_j::L_SHOULDER) = mpi.row(mpi_j::RIGHT_SHOULDER) - up * unit * 0.25 + forward * unit * 0.1;
    out.row(smpl_j::R_ELBOW) = mpi.row(mpi_j::LEFT_ELBOW);
    out.row(smpl_j::L_ELBOW) = mpi.row(mpi_j::RIGHT_ELBOW);
    out.row(smpl_j::R_WRIST) = mpi.row(mpi_j::LEFT_WRIST);
    out.row(smpl_j::L_WRIST) = mpi.row(mpi_j::RIGHT_WRIST);
    out.row(smpl_j::R_HAND) = mpi.row(mpi_j::LEFT_WRIST) * 1.4 - mpi.row(mpi_j::LEFT_ELBOW) * 0.4;
    out.row(smpl_j::L_HAND) = mpi.row(mpi_j::RIGHT_WRIST) * 1.4 - mpi.row(mpi_j::RIGHT_ELBOW) * 0.4;
    out.row(smpl_j::R_COLLAR) = mpi.row(mpi_j::LEFT_SHOULDER) * 0.75 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.25 - up * unit * 0.5;
    out.row(smpl_j::L_COLLAR) = mpi.row(mpi_j::LEFT_SHOULDER) * 0.25 + mpi.row(mpi_j::RIGHT_SHOULDER) * 0.75 - up * unit * 0.5;
    out.row(smpl_j::R_FOOT) = mpi.row(mpi_j::LEFT_ANKLE) * 1.1 -  mpi.row(mpi_j::LEFT_KNEE) * 0.1 + forward * unit;
    out.row(smpl_j::L_FOOT) = mpi.row(mpi_j::RIGHT_ANKLE) * 1.1 -  mpi.row(mpi_j::RIGHT_KNEE) * 0.1 + forward * unit;

    for (int i = 0; i < out.rows(); ++i) {
        out.row(i) -= forward * unit * 0.2;
        if (out.row(i).x() < -1e10 || std::isnan(out.row(i).x())) {
            out.row(i).x() = NAN;
        }
    }
}

int main(int argc, char ** argv) {
    google::InitGoogleLogging(argv[0]);

    printf("Welcome to OpenARK v %s Demo\n\n", VERSION);
    // seed the rng
    srand(time(NULL));

    const std::string IMG_PATH = "C:\\proj\\openark\\OpenARK_dataset\\human-basic-rgb-D435\\capture_14.yml";
    // gender-neutral model
    const std::string HUMAN_MODEL_PATH = "C:/datasets/human/SMPL/models/basicModel_neutral_lbs_10_207_0_v1.0.0/";
    // male model
    //const std::string HUMAN_MODEL_PATH = "C:/datasets/human/SMPL/models/basicModel_m_lbs_10_207_0_v1.0.0/";

    // female model
    //const std::string HUMAN_MODEL_PATH = "C:/dataSets/human/SMPL/models/basicModel_f_lbs_10_207_0_v1.0.0/";

    const std::vector<std::string> SHAPE_KEYS = { "shape000.pcd", "shape001.pcd",

                                           "shape002.pcd", "shape003.pcd", "shape004.pcd", "shape005.pcd", "shape006.pcd", "shape007.pcd",
                                           "shape008.pcd", "shape009.pcd"
    };


    cv::Mat xyzMap, rgbMap;
    std::string imgPath = argc > 1 ? argv[1] : IMG_PATH;
    if (!boost::filesystem::exists(imgPath)) {
        std::cerr << "Image not found! Exiting...\n";
        std::exit(0);
    }
    cv::FileStorage fs2(imgPath, cv::FileStorage::READ);
    fs2["xyz_map"] >> xyzMap;
    fs2["rgb_map"] >> rgbMap;
    fs2.release();

    //xyzMap = xyzMap(cv::Rect(0,0, xyzMap.cols, 420));
    //rgbMap = rgbMap(cv::Rect(0,0, xyzMap.cols, 410));

    // joint estimation using CNN
    std::vector<cv::Point> rgbJoints;
    estimatePoseRGB(rgbMap, xyzMap, rgbJoints, 0.3);

    // segmentation using agglomerate clustering
    cv::Mat out;
    segmentAvatar(xyzMap, { rgbJoints[OpenPoseMPIJoint::LEFT_HIP], rgbJoints[OpenPoseMPIJoint::RIGHT_HIP], 
                            rgbJoints[OpenPoseMPIJoint::LEFT_SHOULDER],  rgbJoints[OpenPoseMPIJoint::RIGHT_SHOULDER]}, out);

    // convert to PCL point cloud
    auto humanCloudRaw = util::toPointCloud<pcl::PointXYZ>(out, true, true);
    auto humanCloud = denoisePointCloud(humanCloudRaw); // denoise and downsample

    auto viewer = Visualizer::getPCLVisualizer();
    auto vp0 = Visualizer::createPCLViewport(0, 0, 0.7, 1), vp1 = Visualizer::createPCLViewport(0.7, 0, 1, 1);
    HumanAvatar::EigenCloud_T xyzJoints;
    toSMPLJoints(out, rgbJoints, xyzJoints);

    // show images
    for (size_t n = 0; n < rgbJoints.size(); ++n) {
        cv::circle(out, rgbJoints[n], 8, cv::Scalar(0, 255, 255), -1);
        cv::putText(out, std::to_string(n), rgbJoints[n], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 0));
    }

    cv::imshow("Fill", out);
    cv::Mat xyzVis;
    Visualizer::visualizeXYZMap(xyzMap, xyzVis, 2.5f);
    cv::imshow("Depth Map", xyzVis);
    cv::imshow("RGB", rgbMap);

    HumanAvatar ava(HUMAN_MODEL_PATH, SHAPE_KEYS, 2);

    // initialize avatar position
    ava.setCenterPosition(util::cloudCenter(humanCloudRaw));
    ava.update();

    /*
    // print out joints (debug)
    for (int i = 0; i < xyzJoints.rows(); ++i) {
        auto r = xyzJoints.row(i);
        cout << r.x() << "\t" << r.y() << "\t" << r.z() << "\n";
    }
    cout << "\n";
    */
    ava.alignToJoints(xyzJoints);
    ava.update();

    const std::string MODEL_CLOUD_NAME = "model_cloud", DATA_CLOUD_NAME = "data_cloud";

    // visualize
    viewer->addPointCloud<pcl::PointXYZ>(humanCloud, DATA_CLOUD_NAME, vp0);
    std::cout << "Data (Human) Points: " << humanCloud->size() << ". "
              << "Model (Avatar) Points: " << ava.getCloud()->size() << "\n";

    viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME, vp0);
    ava.fit(humanCloud);
    ava.visualize(viewer, "o1_ava_", vp1);
    ava.visualize(viewer, "ava_", vp0);
    viewer->removePointCloud(MODEL_CLOUD_NAME, vp0);
    viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME, vp0);
    viewer->addPointCloud<HumanAvatar::Point_T>(ava.getCloud(), MODEL_CLOUD_NAME + "_o1", vp1);
    /*
    for (int i = 0; i < xyzJoints.rows(); ++i) {
        auto pt = xyzJoints.row(i);
        pcl::PointXYZ pclPt(pt.x(), pt.y(), pt.z());
        viewer->addText3D(std::to_string(i), pclPt, 0.1, 0, 0, 255, "jnt_lab__" + std::to_string(i));
        viewer->addSphere(pclPt, 0.04, 1.0, 0, 0, "jnt__" + std::to_string(i));
    }*/

    viewer->spinOnce();
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
