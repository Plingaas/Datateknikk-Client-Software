//
// Created by Pling on 21/11/2024.
//

#ifndef LIDARRECEIVER_HPP
#define LIDARRECEIVER_HPP
#include "TCPConnector.hpp"
#include <thread>
#include <queue>
#include <vector>
#include <iostream>
#include <cstring>
#include "CSVWriter.hpp"
#include "DataTypes.hpp"


class LidarReceiver {
public:
    using NewFrameHandler = std::function<void(LidarFrame& frame)>;
    void setOnNewFrameHandler(NewFrameHandler handler) {new_frame_handler = std::move(handler);};

    LidarReceiver(const std::string& _ip, uint16_t _port);
    void deserializeLidarData(LidarFrame& data, const std::vector<unsigned char>& buffer, int points);

private:
    NewFrameHandler new_frame_handler;

    std::vector<uint8_t> SOF_ = {0xd8, 0xe3, 0xa7};
    std::vector<uint8_t> EOF_ = {0x7a, 0x3e, 0x8d};
    std::string ip;
    uint16_t port;
    std::queue<std::pair<long long, std::vector<uint8_t>>> packets;
    std::unique_ptr<TCPConnector> client;
    std::thread receiverThread;
    std::thread handleThread;
    std::unique_ptr<CSVWriter> logger;
};



#endif //LIDARRECEIVER_HPP
