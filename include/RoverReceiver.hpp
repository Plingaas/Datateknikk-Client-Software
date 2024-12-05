//
// Created by Pling on 04/11/2024.
//


#ifndef STATERECEIVER_HPP
#define STATERECEIVER_HPP
#include "TCPConnector.hpp"
#include <thread>
#include <queue>
#include <iostream>
#include <fstream>
#include <chrono>
#include "CSVWriter.hpp"
#include <mutex>
#include "DataTypes.hpp"

class RoverReceiver {

public:
    using NewFrameHandler = std::function<void(std::shared_ptr<RoverFrame>)>;
    void setOnNewFrameHandler(NewFrameHandler handler) {new_frame_handler = std::move(handler);};

    std::mutex m;
    RoverReceiver(std::string& _ip, uint16_t _port);

    [[nodiscard]] RoverFrame getFrame() const {return *state;}
private:
    NewFrameHandler new_frame_handler;
    std::string ip;
    uint16_t port;
    std::queue<std::pair<long long, std::vector<uint8_t>>> packets;
    std::unique_ptr<TCPConnector> client;
    std::thread receiverThread;
    std::thread handleThread;
    std::unique_ptr<CSVWriter> logger;
    std::shared_ptr<RoverFrame> state;
    std::shared_ptr<RoverFrame> parse(const std::vector<uint8_t> &bytes);
};
#endif //STATERECEIVER_HPP
