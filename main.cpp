
#include "CameraReceiver.hpp"
#include <chrono>

int main() {

    std::string eduroamIP("10.24.85.223");
    std::string ip = eduroamIP;

    uint16_t cameraPort = 8552;

    // Receive data
    CameraReceiver camera(ip, cameraPort);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}