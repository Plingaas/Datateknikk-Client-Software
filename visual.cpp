#include "threepp/threepp.hpp"
#include <iostream>

#include "simple_socket/TCPSocket.hpp"
#include <queue>
#include <thread>
#include <mutex>
#include <cstring>
#include "StateReceiver.hpp"
#include "threepp/input/KeyListener.hpp"
#include "threepp/core/Object3D.hpp"
#include <fstream>
#include <chrono>
#include <LidarReceiver.hpp>
#include "CameraReceiver.hpp"

class KeyController : public threepp::KeyListener {
public:
    explicit KeyController(threepp::Object3D& object_, std::unique_ptr<SimpleConnection> connection) : object(&object_), connection(std::move(connection)) {};
    std::unique_ptr<SimpleConnection> connection;
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

int main() {

    std::string eduroamIP("10.24.87.46");
    std::string hotspotIP("10.42.0.1");
    std::string ip = eduroamIP;
    uint16_t cameraPort = 8552;
    uint16_t roverPort = 9996;
    CameraReceiver cameraReceiver(ip, cameraPort);
    StateReceiver receiver(ip, roverPort);

    threepp::Canvas canvas("lidar_visual");
    threepp::GLRenderer renderer(canvas.size());
    renderer.setClearColor(threepp::Color::white);
    auto camera = threepp::PerspectiveCamera::create();
    camera->position.z = 35;
    threepp::OrbitControls controls{*camera, canvas};
    controls.enableKeys = false;
    auto scene = threepp::Scene::create();
    auto material = threepp::MeshBasicMaterial::create();
    auto geometry = threepp::CircleGeometry::create(0.05);
    material -> color = threepp::Color::red;
    std::vector<std::shared_ptr<threepp::Mesh>> sircles;

    for (int i = 0; i < 130; i++) {
        auto visual = threepp::Mesh::create(geometry, material);
        visual ->position.x = threepp::math::randFloat(-16,16);
        visual ->position.y = threepp::math::randFloat(-16,16);
        sircles.emplace_back(visual);
        scene->add(visual);
    }

    std::queue<std::vector<std::pair<double, double>>> queue;


    uint16_t lidarServerPort = 9998;
    LidarReceiver lidarReceiver(ip, lidarServerPort);

    uint16_t commandPort = 5853;
    TCPServer commandServer(commandPort);
    auto commandConnection = commandServer.accept();
    threepp::Object3D keyObject;
    scene->add(keyObject);
    KeyController keyListener(keyObject, std::move(commandConnection));
    canvas.addKeyListener(keyListener);

    int index = 0;
    int driveIndex = 0;
    canvas.animate([&] {

        if (keyListener.leftPressed) {
            keyListener.bytesToSend.push_back(0x02);
            keyListener.connection->write(keyListener.bytesToSend);
            keyListener.bytesToSend.clear();
        }
        if (keyListener.rightPressed) {
            keyListener.bytesToSend.push_back(0x03);
            keyListener.connection->write(keyListener.bytesToSend);
            keyListener.bytesToSend.clear();
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
     });

}