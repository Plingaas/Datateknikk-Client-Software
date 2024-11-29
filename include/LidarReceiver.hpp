//
// Created by Pling on 21/11/2024.
//

#ifndef LIDARRECEIVER_HPP
#define LIDARRECEIVER_HPP
#include "simple_socket/TCPSocket.hpp"
#include <thread>
#include <queue>
#include <vector>
#include <iostream>
#include <cstring>
#include "CSVWriter.hpp"
using namespace simple_socket;

class LidarReceiver {
private:
    std::string ip;
    uint16_t port;
    std::queue<std::pair<long long, std::vector<uint8_t>>> packets;
    std::unique_ptr<TCPClientContext> client;
    std::thread receiverThread;
    std::thread handleThread;
    std::unique_ptr<CSVWriter> logger;
public:
    LidarReceiver(const std::string& _ip, uint16_t _port);
    std::vector<std::pair<double, double>> deserializeLidarData(const std::vector<unsigned char>& buffer);

};



#endif //LIDARRECEIVER_HPP
