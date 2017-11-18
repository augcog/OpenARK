#include "stdafx.h"

#include "Hand.h"

#include "Util.h"
#include "Visualizer.h"

Hand::Hand()
{

}

Hand::Hand(cv::Vec3f centroid_xyz, cv::Point2i centroid_ij, std::vector<cv::Vec3f> fingers_xyz,
    std::vector<cv::Point2i> fingers_ij, std::vector<cv::Vec3f> defects_xyz, std::vector<cv::Point2i> defects_ij)
{
    this->centroid_xyz = centroid_xyz;
    this->centroid_ij = centroid_ij;
    this->fingers_xyz = fingers_xyz;
    this->fingers_ij = fingers_ij;
    this->defects_xyz = defects_xyz;
    this->defects_ij = defects_ij;
}

//Hand::Hand(cv::Mat xyzMap, float angle_threshhold, int cluster_thresh)
//{
//    CLUSTER_THRESHOLD = cluster_thresh * (float)xyzMap.cols / 320.0;
//    ANGLE_THRESHHOLD = angle_threshhold;
//}
//

Hand::~Hand()
{

}

int Hand::getNumFingers() {
    return (int)fingers_xyz.size();
}

bool Hand::touchObject(std::vector<double> &equation, const double threshold)
{
    if (equation.size() == 0)
    {
        return false;
    }

    for (int i = 0; i < fingers_xyz.size(); i++)
    {
        double x = fingers_xyz[i][0];
        double y = fingers_xyz[i][1];
        double z = fingers_xyz[i][2];

        if (z == 0)
        {
            return false;
        }

        double z_hat = equation[0] * x + equation[1] * y + equation[2];
        double r_squared = (z - z_hat) * (z - z_hat);

        if (r_squared < threshold)
        {
            return true;
        }
    }

    return false;
}