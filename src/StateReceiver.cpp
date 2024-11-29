//
// Created by Pling on 04/11/2024.
//

#include <StateReceiver.hpp>
#include <iostream>
#include <cstring>
#include <mutex>

StateReceiver::StateReceiver(std::string &_ip, uint16_t _port) {
    ip = _ip;
    port = _port;
    client = std::make_unique<TCPClientContext>();
    state = std::make_shared<RoverState>();
    logger = std::make_unique<CSVWriter>("data/RoverData.csv", "time,pitch,roll,yaw,ax,ay,az,gx,gy,gz,x,y,vx,vy");

    receiverThread = std::thread([this, _ip, _port] {
        while (true) {
            std::cout << "Trying to connect to rover state stream." << std::endl;
            auto connection = client->connect(_ip, _port);
            try {
                std::cout << "Connected to server" << std::endl;
                while (true) {
                    std::pair<long long, std::vector<uint8_t>> data;
                    std::vector<uint8_t> buffer(52);
                    connection->read(buffer);
                    auto now = std::chrono::system_clock::now();
                    auto msSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                    packets.emplace(msSinceEpoch, buffer);
                }
            } catch (...) {
                std::cerr << "Client connection error?" << std::endl;
            }
        }
    });

    handleThread = std::thread([this] {
        int dataCount = 0;
        while (true) {
           if (!packets.empty()) {
               state = parse(packets.front().second);
               logger->log(packets.front().first, state->getData());
               packets.pop();

               if (dataCount % 10 == 0) {
                   std::cout <<  "Rover data line count: " << dataCount++ << std::endl;
               }
           }
        }
    });

    receiverThread.detach();
    handleThread.detach();
}

void StateReceiver::handleMessage(std::vector<uint8_t>& buffer) {

};

std::shared_ptr<RoverState> StateReceiver::parse(const std::vector<uint8_t> &bytes) {

    float data[13];
    for (int i = 0; i < 13; i++) {
        std::memcpy(&data[i], &bytes[i*sizeof(float)], sizeof(float));
    }
    return std::make_shared<RoverState>(data);
};

