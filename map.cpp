//
// Created by Pling on 03/12/2024.
//
#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <thread>
#include <fstream>
#include <map>
#include "GridMap.hpp"
#include "LidarReceiver.hpp"
#include "RoverReceiver.hpp"
#include "CameraReceiver.hpp"
#include "threepp/threepp.hpp"

class KeyController : public threepp::KeyListener {
public:
    explicit KeyController(threepp::Object3D& object_, std::unique_ptr<simple_socket::SimpleConnection> connection) : object(&object_), connection(std::move(connection)) {};
    std::unique_ptr<simple_socket::SimpleConnection> connection;
    threepp::Object3D *object;
    bool rightPressed = false;
    bool leftPressed = false;
    bool upPressed = false;
    bool downPressed = false;
    std::vector<uint8_t> bytesToSend{};

    void onKeyPressed(threepp::KeyEvent evt) override {
        std::cout << "onKeyPressed" << std::endl;
        if (evt.key == threepp::Key::UP) {
            bytesToSend.push_back(0x01);
            upPressed = true;
        }
        else if (evt.key == threepp::Key::LEFT) {
            bytesToSend.push_back(0x02);
            leftPressed = true;
        }
        else if (evt.key == threepp::Key::RIGHT) {
            bytesToSend.push_back(0x03);
            rightPressed = true;
        }
        else if (evt.key == threepp::Key::DOWN) {
            bytesToSend.push_back(0x04);
            downPressed = true;
        }
        connection->write(bytesToSend);
        bytesToSend.clear();
    }

    void onKeyReleased(threepp::KeyEvent evt) override {

        if (evt.key == threepp::Key::UP) {
            bytesToSend.push_back(0x05);
            upPressed = false;
        }
        else if (evt.key == threepp::Key::LEFT) {
            leftPressed = false;
        }
        else if (evt.key == threepp::Key::RIGHT) {
            rightPressed = false;
        }
        else if (evt.key == threepp::Key::DOWN) {
            bytesToSend.push_back(0x05);
            downPressed = false;
        }
        connection->write(bytesToSend);
        bytesToSend.clear();
    }
};


int getGridIndex(double value, int gridSize, int worldSize) {
    return static_cast<int>(value / (worldSize / (static_cast<float>(gridSize)) ));
}


std::vector<std::pair<RoverFrame, LidarFrame>> readDataset(const std::string& filename, int numLines, int gridSize, int mapSize) {
    std::cout << "Reading " << filename << " with " << numLines << " lines." << std::endl;
    std::vector<std::pair<RoverFrame, LidarFrame>> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error: Could not open file " + filename);
    }

    std::string line;
    int lineCount = 0;
    while (std::getline(file, line) && lineCount < numLines) {
        lineCount++;

        // Split line by semicolon
        std::stringstream ss(line);
        std::string t, posXStr, posYStr, headingStr, lidarData;
        std::getline(ss, t, ';');
        std::getline(ss, posXStr, ';');
        std::getline(ss, posYStr, ';');
        std::getline(ss, headingStr, ';');
        std::getline(ss, lidarData);

        double posX = std::stod(posXStr);
        double posY = std::stod(posYStr);
        double heading = std::stod(headingStr);

        // Apply transformation for the position
        double _posX = posY;
        double _posY = -posX;

        RoverFrame roverFrame;
        roverFrame.yaw = heading;
        roverFrame.x = _posX;
        roverFrame.y = _posY;

        int roverGridX = getGridIndex(_posX, gridSize, mapSize);
        int roverGridY = getGridIndex(_posY, gridSize, mapSize);

        // Parse lidar data (assuming it's formatted as "x1,y1:x2,y2:...")
        LidarFrame lidarFrame;
        std::stringstream lidarStream(lidarData);
        std::string point;
        while (std::getline(lidarStream, point, ':')) {
            std::stringstream pointStream(point);
            double x, y;
            char comma; // To read the comma between x and y

            if (pointStream >> x >> comma >> y) {
                double _x = x * std::cos(heading) - y * std::sin(heading) + _posX;
                double _y = x * std::sin(heading) + y * std::cos(heading) + _posY;
                // Default confidence value set to 0
                lidarFrame.points.push_back({_x, _y});
            } else {
                std::cerr << "Failed to parse lidar point: " << point << std::endl;
            }
        }

        // Create and add frame to the vector
        std::pair<RoverFrame, LidarFrame> pair(roverFrame, lidarFrame);
        data.push_back(pair);
    }

    file.close();
    return data;
}



int main() {
    // Create a 500x500 black image

    int gridSize = 300;
    int mapSize = 30;
    int obstacleThreshold = 7;

    std::string hotspotIP("10.42.0.1");
    std::string eduroamIP("10.24.85.223");
    std::string ip = eduroamIP;

    uint16_t cameraPort = 8552;
    uint16_t roverPort = 9996;
    uint16_t lidarPort = 9998;

    // Receive data
    CameraReceiver cameraReceiver(ip, cameraPort);
    RoverReceiver rover(ip, roverPort);
    LidarReceiver lidar(ip, lidarPort);

    GridMap map(gridSize, mapSize);
    map.setObstacleThreshold(obstacleThreshold);

    Frame frame;
    std::atomic<bool> newFrame = false;
    std::mutex m;
    lidar.setOnNewFrameHandler([&](const LidarFrame& lidarFrame) {
        std::unique_lock<std::mutex> lockRover(rover.m);
        RoverFrame roverFrame = rover.getFrame();
        lockRover.unlock();

        std::lock_guard<std::mutex> lockMain(m);
        frame = Frame(roverFrame, lidarFrame);
        newFrame = true;
    });


    std::queue<float> targetQueue;
    map.setNewPathHandler([&m, &targetQueue](Cell& current, Cell& target) {

        int dx = current.x - target.x;
        int dy = current.y - target.y;

        float wantedHeading = std::atan2(dy, dx) * 180.0f/3.14159265 - 179.9f;
        if (wantedHeading < 0) {
            wantedHeading += 360;
        }
        std::cout << wantedHeading << std::endl;
        std::lock_guard<std::mutex> lockMain(m);
        targetQueue.push(wantedHeading);
    });

    uint16_t commandPort = 5853;
    simple_socket::TCPServer commandServer(commandPort);
    auto commandConnection = commandServer.accept();

    while (true) {
        if (newFrame) {
            map.update(frame);
            newFrame = false;
        }

        if (!targetQueue.empty()) {
            std::vector<uint8_t> buffer(5);
            buffer[0] = 0x06;
            std::memcpy(buffer.data()+1, &targetQueue.front(), sizeof(float));
            commandConnection->write(buffer);
            targetQueue.pop();
        }
    }

    /*threepp::Canvas canvas("KeyInput");
    canvas.setSize({100, 100});
    threepp::GLRenderer renderer(canvas.size());
    auto camera = threepp::PerspectiveCamera::create();
    auto scene = threepp::Scene::create();

    uint16_t commandPort = 5853;
    simple_socket::TCPServer commandServer(commandPort);
    auto commandConnection = commandServer.accept();

    threepp::Object3D keyObject;
    scene->add(keyObject);
    KeyController keyListener(keyObject, std::move(commandConnection));
    canvas.addKeyListener(keyListener);

    int index = 0;
    int driveIndex = 0;
    int turnIndex = 0;
    canvas.animate([&] {

        if (keyListener.leftPressed) {
            if (turnIndex == 20) {
                keyListener.bytesToSend.push_back(0x02);
                keyListener.connection->write(keyListener.bytesToSend);
                keyListener.bytesToSend.clear();
                turnIndex = 0;
            } else {
                turnIndex++;
            }
        }
        if (keyListener.rightPressed) {
            if (turnIndex == 20) {
                keyListener.bytesToSend.push_back(0x03);
                keyListener.connection->write(keyListener.bytesToSend);
                keyListener.bytesToSend.clear();
                turnIndex = 0;
            } else {
                turnIndex++;
            }
        }
        if (keyListener.upPressed) {
            if (driveIndex == 20) {
                keyListener.bytesToSend.push_back(0x01);
                keyListener.connection->write(keyListener.bytesToSend);
                keyListener.bytesToSend.clear();
                driveIndex = 0;
            } else {
                driveIndex++;
            }
        }
        if (keyListener.downPressed) {
            if (driveIndex == 20) {
                keyListener.bytesToSend.push_back(0x04);
                keyListener.connection->write(keyListener.bytesToSend);
                keyListener.bytesToSend.clear();
                driveIndex = 0;
            } else {
               driveIndex++;
            }
        }

        renderer.render(*scene, *camera);
        if (newFrame) {
            map.update(frame);
            newFrame = false;
        }
     });*/
    return 0;
}

