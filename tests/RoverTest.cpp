//
// Created by Pling on 05/12/2024.
//
//
// Created by Pling on 05/12/2024.
//
#include "RoverReceiver.hpp"
#include "DataTypes.hpp"
#include "opencv2/opencv.hpp"
int main() {

    std::string ip = "10.24.85.223";
    uint16_t port = 9996;
    RoverReceiver rover(ip, port);


    rover.setOnNewFrameHandler([&](std::shared_ptr<RoverFrame> roverFrame) {
        std::cout << roverFrame->yaw << std::endl;
    });


    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
