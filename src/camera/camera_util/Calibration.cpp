#include "stdafx.h"
#include "Version.h"
#include "camera/camera_util/Calibration.h"
#include "util/Util.h"

namespace ark {
    void Calibration::XYZToUnity(DepthCamera& depth_cam, int num_boards, int board_w, int board_h)
    {
        auto board_sz = cv::Size(board_w, board_h);
        auto board_n = board_w * board_h;
        std::vector<Point2f> cornersAmp; // Corners of amplitude image
        std::vector<Vec3f> cornersXYZ; // Corners of the depth image
        std::vector<std::vector<Vec3f>> XYZ_points;
        std::vector<Vec3f> upper_left;
        upper_left.push_back(Vec3f(-0.05, 0.03, 0.40));
        upper_left.push_back(Vec3f(0.05, 0.03, 0.40));
        upper_left.push_back(Vec3f(0.0, 0.08, 0.35));
        upper_left.push_back(Vec3f(0.0, -0.02, 0.35));
        auto Unity_points = prepareUnityData(upper_left, 0.03, board_h, board_w);
        auto success = 0;

        // Collect data for calibration
        while (success < num_boards)
        {
            cornersAmp.clear();
            cornersXYZ.clear();
            bool found1 = false;
            depth_cam.nextFrame();
            auto xyzMap = depth_cam.getXYZMap();

            cv::Mat ampGray;
            cv::normalize(depth_cam.getAmpMap(), ampGray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::equalizeHist(ampGray, ampGray);
            cv::resize(ampGray, ampGray, cv::Size(ampGray.cols * 4, ampGray.rows * 4));

            // Sharpen amplitude image
            cv::Mat unsharp_mask;
            cv::GaussianBlur(ampGray, unsharp_mask, cv::Size(5, 5), 5);
            cv::addWeighted(ampGray, 1.5, unsharp_mask, -0.5, 0, ampGray);
            cv::imshow("Gray Amp", ampGray);

            // Find chessboards on amplitude
            found1 = findChessboardCorners(ampGray, board_sz, cornersAmp, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
            if (found1) {
                cv::cornerSubPix(ampGray, cornersAmp, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                auto ampRGB = ampGray.clone();
                cv::cvtColor(ampRGB, ampRGB, CV_GRAY2BGR);
                cv::drawChessboardCorners(ampRGB, board_sz, cornersAmp, found1);
                cv::imshow("Gray Corners", ampRGB);

                for (auto i = 0; i < cornersAmp.size(); i++)
                {
                    cornersAmp[i].x = cornersAmp[i].x / 4;
                    cornersAmp[i].y = cornersAmp[i].y / 4;
                }

                for (auto i = 0; i < cornersAmp.size(); i++)
                {
                    auto xyz = util::averageAroundPoint(xyzMap, cv::Point2i(cornersAmp[i].x, cornersAmp[i].y), 5);
                    cv::Point3f pt;
                    pt.x = xyz[0]; pt.y = xyz[1], pt.z = xyz[2];
                    if (pt.z == 0) {
                        continue;
                    }
                    cornersXYZ.push_back(pt);
                }

                int c = cv::waitKey(1);
                if (c == ' ')
                {
                    success++;
                    XYZ_points.push_back(cornersXYZ);
                    printf("%u points recorded!\n", (unsigned) cornersXYZ.size());
                }
            }

            auto c = cv::waitKey(1);
            if (c == 'q' || c == 'Q' || c == 27)
            {
                break;
            }
        }

        /**** Perform calculations ****/
        Calibration::writeDataToFile(Unity_points, 4, 3, "Unity.txt");
        Calibration::writeDataToFile(XYZ_points, 4, 3, "XYZ.txt");
        float x_input[3][48];
        float y_input[3][48];
        auto count = 0;
        for (auto i = 0; i < XYZ_points.size(); i++)
        {
            for (auto v = 0; v < XYZ_points[i].size(); v++)
            {
                x_input[0][count] = XYZ_points[i][v][0];
                x_input[1][count] = XYZ_points[i][v][1];
                x_input[2][count] = XYZ_points[i][v][2];
                y_input[0][count] = Unity_points[i][v][0];
                y_input[1][count] = Unity_points[i][v][1];
                y_input[2][count] = Unity_points[i][v][2];
                count++;
            }
        }

        auto x = cv::Mat(3, 48, CV_32FC1, &x_input); //XYZ
        auto y = cv::Mat(3, 48, CV_32FC1, &y_input); //Unity
        cv::Mat r, t;
        computeRT(x, y, &r, &t);
        cv::FileStorage fs("RT_Transform.txt", cv::FileStorage::WRITE);
        fs << "R" << r;
        fs << "T" << t;
        fs.release();
    }

    void Calibration::computeRT(cv::Mat x, cv::Mat y, cv::Mat *R, cv::Mat *t)
    {

        cv::Mat x_mean, y_mean, H;
        cv::reduce(x, x_mean, 1, CV_REDUCE_AVG);
        cv::reduce(y, y_mean, 1, CV_REDUCE_AVG);

        for (auto i = 0; i < x.cols; i++)
        {
            H = H + (x.col(i) - x_mean) * ((y.col(i) - y_mean).t());
        }

        cv::Mat u, s, v;
        cv::SVD::compute(H, s, u, v);
        u = -1 * u;
        v = -1 * v.t();
        *R = v * u.t();

        if (cv::determinant(*R) > 0)
        {
            v.col(2) = -1 * v.col(2);
            *R = v * u.t();
        }
        *t = -1 * *R * x_mean + y_mean;
    }

    void Calibration::XYZToRGB(DepthCamera* depth_cam, RGBCamera* rgb_cam, int num_boards, int board_w, int board_h)
    {
        return;
    }

    double Calibration::reprojectXYZToUnity(std::vector<std::vector<Vec3f>> XYZ_points, std::vector<std::vector<Vec3f>> Unity_points, Eigen::MatrixXf R, Eigen::MatrixXf T)
    {
        // Return error value if there is a size mismatch
        if (XYZ_points.size() != Unity_points.size())
        {
            return -1.00;
        }

        double error = 0;
        for (auto i = 0; i < XYZ_points.size(); i++)
        {
            for (auto v = 0; v < XYZ_points[i].size(); v++)
            {
                Eigen::MatrixXf pt_xyz(3, 1);
                Eigen::MatrixXf pt_unity(3, 1);
                pt_xyz(0, 0) = XYZ_points[i][v][0];
                pt_xyz(1, 0) = XYZ_points[i][v][1];
                pt_xyz(2, 0) = XYZ_points[i][v][2];
                pt_unity(0, 0) = Unity_points[i][v][0];
                pt_unity(1, 0) = Unity_points[i][v][1];
                pt_unity(2, 0) = Unity_points[i][v][2];

                Eigen::MatrixXf result = R * T * pt_xyz;
                error += abs((result - pt_unity).norm());
            }
        }

        return error;
    }

    double Calibration::reprojectXYZtoRGB()
    {
        return false;
    }

    std::vector<std::vector<Vec3f>> Calibration::prepareUnityData(std::vector<Vec3f> upper_left, float distance, int num_rows, int num_cols)
    {
        std::vector<std::vector<Vec3f>> Unity_points;

        for (auto i = 0; i < upper_left.size(); i++)
        {
            std::vector<Vec3f> points;
            for (auto y = 0; y < num_rows; y++)
            {
                for (auto x = 0; x < num_cols; x++)
                {
                    points.push_back(cv::Vec3f(upper_left[i][0] + x * distance, upper_left[i][1] - y * distance, upper_left[i][2]));
                }
            }
            Unity_points.push_back(points);
        }

        return Unity_points;
    }

    void Calibration::writeDataToFile(std::vector<std::vector<Vec3f>> points, int board_w, int board_h, std::string filename)
    {
        ofstream output_file;
        output_file.open(filename);
        for (auto i = 0; i < points.size(); i++)
        {
            for (auto v = 0; v < points[i].size(); v++)
            {
                if (v%board_w == 0)
                {
                    output_file << "\n";
                }
                output_file << "(" << points[i][v][0] << ", " << points[i][v][1] << ", " << points[i][v][2] << ") ";
            }
            output_file << "\n\n";
        }
        output_file.close();
    }
}
