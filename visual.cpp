#include "threepp/threepp.hpp"
#include <iostream>
#include "simple_socket/TCPSocket.hpp"
#include <queue>
#include <mutex>
#include "RoverReceiver.hpp"
#include "threepp/input/KeyListener.hpp"
#include "threepp/core/Object3D.hpp"
#include <LidarReceiver.hpp>
#include "CameraReceiver.hpp"

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

int main() {

    std::string eduroamIP("10.24.85.223");
    std::string hotspotIP("10.42.0.1");

    std::string ip = eduroamIP;
    uint16_t cameraPort = 8553;
    uint16_t roverPort = 9996;
    uint16_t lidarPort = 9998;

    // Receive data
    CameraReceiver cameraReceiver(ip, cameraPort);
    RoverReceiver receiver(ip, roverPort);
    LidarReceiver lidarReceiver(ip, lidarPort);

        threepp::Canvas canvas("KeyInput");
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
                if (turnIndex == 0) {
                    keyListener.bytesToSend.push_back(0x02);
                    keyListener.connection->write(keyListener.bytesToSend);
                    keyListener.bytesToSend.clear();
                    turnIndex = 0;
                } else {
                    turnIndex++;
                }
            }
            if (keyListener.rightPressed) {
                if (turnIndex == 0) {
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
         });

}