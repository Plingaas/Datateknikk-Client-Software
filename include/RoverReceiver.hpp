//
// Created by Pling on 04/11/2024.
//


#ifndef STATERECEIVER_HPP
#define STATERECEIVER_HPP
#include "simple_socket/TCPSocket.hpp"
#include <thread>
#include <queue>
#include <iostream>
#include <fstream>
#include <chrono>
#include "CSVWriter.hpp"

using namespace simple_socket;

struct RoverState {
    float pitch, roll, yaw = 0;
    float ax, ay, az = 0;
    float gx, gy, gz = 0;
    float x, y = 0;
    float vx, vy = 0;

    RoverState() {};

    RoverState(const float* data) {
        pitch = data[0];
        roll = data[1];
        yaw = data[2];
        ax = data[3];
        ay = data[4];
        az = data[5];
        gx = data[6];
        gy = data[7];
        gz = data[8];
        x = data[9];
        y = data[10];
        vx = data[11];
        vy = data[12];
    };

    std::vector<float> getData() {
        return {pitch, roll, yaw, ax, ay, az, gx, gy, gz, x, y, vx, vy};
    }

    void print() {
        std::cout << "Pitch: " << pitch << " | Roll: " << roll << " | Yaw: " << yaw << std::endl;
        std::cout << "Ax: " << ax << " | AY: " << ay << " | AZ: " << az << std::endl;
        std::cout << "GX: " << gx << " | GY: " << gy << std::endl;
        std::cout << "x: " << x << " | y: " << y << std::endl;
        std::cout << "vx: " << vx << " | vy: " << vy << std::endl << std::endl;
    }
};

class RoverReceiver {
private:
    std::string ip;
    uint16_t port;
    std::queue<std::pair<long long, std::vector<uint8_t>>> packets;
    std::unique_ptr<TCPClientContext> client;
    std::thread receiverThread;
    std::thread handleThread;
    std::unique_ptr<CSVWriter> logger;
    std::shared_ptr<RoverState> state;
    std::shared_ptr<RoverState> parse(const std::vector<uint8_t> &bytes);

public:

    RoverReceiver(std::string& _ip, uint16_t _port);
    void handleMessage(std::vector<uint8_t>& packet);
    void listen();
};
#endif //STATERECEIVER_HPP
