//
// Created by Pling on 05/12/2024.
//
#include "LidarReceiver.hpp"
#include "DataTypes.hpp"
#include "opencv2/opencv.hpp"
int main() {

    std::string ip = "10.24.85.223";
    uint16_t port = 9998;
    LidarReceiver lidar(ip, port);


    lidar.setOnNewFrameHandler([&](const LidarFrame& lidarFrame) {
        auto image = cv::Mat(1000, 1000, CV_8UC3);

        for (auto point : lidarFrame.points) {
            if (point.first == 0 && point.second == 0) continue;
            int x = point.first*40;
            int y = point.second*40;
            cv::circle(image, {500+x, 500+y}, 1, cv::Scalar(0, 0, 255), 1);
        }
        cv::imshow("frame", image);
        cv::waitKey(1);
    });


    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
