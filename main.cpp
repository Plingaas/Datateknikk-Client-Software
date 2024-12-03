
#include "RoverReceiver.hpp"
#include "LidarReceiver.hpp"
#include "CameraReceiver.hpp"
#include "KeyboardController.hpp"
#include <chrono>

int main() {

    std::string eduroamIP("10.24.85.223");
    std::string hotspotIP("10.42.0.1");

    std::string ip = eduroamIP;
    uint16_t cameraPort = 8552;
    uint16_t roverPort = 9996;
    uint16_t lidarPort = 9998;

    // Receive data
    CameraReceiver cameraReceiver(ip, cameraPort);
    RoverReceiver receiver(ip, roverPort);
    LidarReceiver lidarReceiver(ip, lidarPort);

    // Command
    uint16_t commandPort = 5853;
    KeyboardController controller(commandPort);
    //AutonomousController controller;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}