#include "stdafx.h"
#include "Version.h"
#include "NetworkStereoCamera.h"
#include "Visualizer.h"
#include "ProtobufImage.pb.h"
#include "NetworkLib.h"

namespace ark {
    NetworkStereoCamera::NetworkStereoCamera(StereoCalibration::Ptr calib, int port,
        NetworkStereoCamera::StereoUpdateCallback stereoUpdateCallback, SGBMConfig::Ptr sgbmConf)
        : StereoCamera(calib, sgbmConf), udpServer(new NetworkLib::Server(port)), port(port), updCallback(stereoUpdateCallback) {
    }

    // overrided model name
    const std::string NetworkStereoCamera::getModelName() const { return "NetworkStereo"; }

    int NetworkStereoCamera::numClients() const
    {
        return static_cast<int>(udpServer->GetClientCount());
    }

    uint32_t NetworkStereoCamera::getClientByIndex(int index) const
    {
        return udpServer->GetClientIdByIndex(index);
    }

    std::vector<uint32_t> NetworkStereoCamera::getClients(uint32_t client) const
    {
        return udpServer->GetClients();
    }

    void NetworkStereoCamera::sendToClient(const std::string & message, uint32_t client, bool base_64) const
    {
        if (base_64) {
            std::string encoded;
            NetworkLib::Util::Base64Encode(message, &encoded);
            udpServer->SendToClient(encoded, client);
        }
        else {
            udpServer->SendToClient(message, client);
        }
    }

    void NetworkStereoCamera::sendToAllClients(const std::string & message, bool base_64) const
    {
        std::string encoded;
        if (base_64) {
            NetworkLib::Util::Base64Encode(message, &encoded);
        }
        else {
            encoded = message;
        }
        for (uint32_t client : udpServer->GetClients()) {
            sendToClient(encoded, client, false);
        }
    }

    bool NetworkStereoCamera::frameReady() const
    {
        return !latestFrame.xyzMap.empty();
    }

    NetworkStereoCamera::Frame NetworkStereoCamera::getLatestFrame() const
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        return latestFrame;
    }

    void NetworkStereoCamera::setStereoUpdateCallback(StereoUpdateCallback callback)
    {
        updCallback = callback;
    }

    void NetworkStereoCamera::update(cv::Mat & xyz_map, cv::Mat & rgb_map, cv::Mat & ir_map, 
            cv::Mat & amp_map, cv::Mat & flag_map) {
        // on camera thread; block until next frame received
        while (true) {
            while (!udpServer->HasMessages()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            std::string message;
            Frame frame;

            // process only latest frame
            while (udpServer->HasMessages()) {
                std::tie(message, frame.clientID) = udpServer->PopMessage();
            }

            // receive image
            protob::Image image;
            image.ParseFromString(message);
            std::string imgProtoStr = image.image_string();
            std::vector<unsigned char> vectordata(imgProtoStr.begin(), imgProtoStr.end());

            // fill in the frame
            cv::Mat dataMat(vectordata, true);
            if (dataMat.empty()) continue;

            cv::Mat combined = cv::imdecode(dataMat, 0);
            if (combined.empty()) continue;

            xyz_map = frame.xyzMap = computeDepthSGBM(combined, &frame.leftImage);
            cv::cvtColor(frame.leftImage, rgb_map, cv::COLOR_GRAY2BGR);
            cv::resize(rgb_map, rgb_map, calib->imageSize);

            static int cur_time = 0; // should only be accessed in this thread
            frame.timestamp = ++cur_time;

            // call the callback
            if (updCallback) {
                updCallback(frame);
            }

            std::lock_guard<std::mutex> lock(frameMutex);
            latestFrame = frame;
            break;
        }
    }
}
