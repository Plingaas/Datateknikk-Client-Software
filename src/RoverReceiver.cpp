//
// Created by Pling on 04/11/2024.
//

#include <RoverReceiver.hpp>
#include <iostream>
#include <cstring>
#include <mutex>

RoverReceiver::RoverReceiver(std::string &_ip, uint16_t _port) {
    ip = _ip;
    port = _port;
    client = std::make_unique<TCPConnector>(ip, port, true);
    client->setName("Rover");
    client->setTimeout(1000);
    client->connect();

    //client->setConnectHandler([this] {onConnect();});
    //client->setConnectionLostHandler([this] {onConnectionLost();});
    //client->setDisconnectHandler([this] {onDisconnect();});
    //client->setConnectionRestoredHandler([this] {onConnectionRestored();});

    state = std::make_shared<RoverFrame>();
    logger = std::make_unique<CSVWriter>("data/RoverData.csv", "time,pitch,roll,yaw,ax,ay,az,gx,gy,gz,x,y,vx,vy");

    receiverThread = std::thread([this, _ip, _port] {
        while (true) {
            std::cout << "Trying to connect to rover state stream." << std::endl;
            client->connect();
            try {
                std::cout << "Connected to server" << std::endl;

                std::vector<uint8_t> SOF_ = {0xd8, 0xd9, 0xda};
                uint8_t metaDataSize = SOF_.size() + sizeof(uint64_t);

                while (true) {

                    std::vector<uint8_t> metaDataBuffer(metaDataSize);
                    client->read(metaDataBuffer, metaDataSize);
                    if (!(metaDataBuffer[0] == SOF_[0] &&
                        metaDataBuffer[1] == SOF_[1] &&
                        metaDataBuffer[2] == SOF_[2])) {
                        client->flush();
                        continue;
                    }

                    uint64_t t;
                    std::memcpy(&t, metaDataBuffer.data() + 3, sizeof(uint64_t));
                    std::vector<uint8_t> buffer(52);
                    client->read(buffer, 52);

                    std::lock_guard<std::mutex> lock(m);
                    packets.emplace(t, buffer);
                }
            } catch (...) {
                std::cerr << "Client connection error?" << std::endl;
            }
        }
    });

    handleThread = std::thread([this] {
        while (true) {
           if (!packets.empty()) {
               try {
                   std::lock_guard<std::mutex> lock(m);
                   state = parse(packets.front().second);
                   if (new_frame_handler) new_frame_handler(state);
                   logger->log(packets.front().first, state->getData());
                   packets.pop();
               } catch (std::exception& e) {
                   std::cout << e.what() << std::endl;
               }
           }
        }
    });

    receiverThread.detach();
    handleThread.detach();
}

std::shared_ptr<RoverFrame> RoverReceiver::parse(const std::vector<uint8_t> &bytes) {

    float data[13];
    for (int i = 0; i < 13; i++) {
        std::memcpy(&data[i], &bytes[i*sizeof(float)], sizeof(float));
    }
    return std::make_shared<RoverFrame>(data);
};

