//
// Created by Pling on 21/11/2024.
//

#include "LidarReceiver.hpp"

#include "TCPConnector.hpp"

LidarReceiver::LidarReceiver(const std::string& _ip, uint16_t _port) {
    ip = _ip;
    port = _port;
    client = std::make_unique<TCPConnector>(ip, port, true);
    client->setName("Lidar");
    client->setTimeout(1000);
    client->connect();

    logger = std::make_unique<CSVWriter>("data/LidarData.csv", "time,points");

    receiverThread = std::thread([this] {
        try {
            client->connect();
            std::cout << "Connected to lidar server." << std::endl;
            std::vector<uint8_t> SOF_ = {0xd8, 0xd9, 0xda};
            uint8_t metaDataSize = 10;
            uint8_t SOFSize = SOF_.size();

            while (true) {

                std::vector<uint8_t> metaDataBuffer(metaDataSize + SOFSize);
                client->read(metaDataBuffer, metaDataSize + SOFSize);

                // Check if SOF matches
                if (!(metaDataBuffer[0] == SOF_[0] &&
                    metaDataBuffer[1] == SOF_[1] &&
                    metaDataBuffer[2] == SOF_[2])) {
                    client->flush();
                    continue;
                }

                uint64_t t;
                std::memcpy(&t, metaDataBuffer.data() + 3, sizeof(uint64_t));

                uint16_t pointsInBuffer = metaDataBuffer[11] << 8 | metaDataBuffer[12];
                size_t pointBytes = pointsInBuffer * 2 * sizeof(float);
                std::vector<uint8_t> data(pointBytes);
                client->read(data, pointBytes);

                LidarFrame frame;
                deserializeLidarData(frame, data, pointsInBuffer);

                if (new_frame_handler) new_frame_handler(frame);

                std::vector<float> logData;
                for (auto pair : frame.points) {
                    logData.push_back(pair.first);
                    logData.push_back(pair.second);
                }

                logger->log(t, logData);
            }
        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    });

    receiverThread.detach();
}

void LidarReceiver::deserializeLidarData(LidarFrame& frame, const std::vector<unsigned char> &buffer, int points) {

    size_t bytes = points * 2 * sizeof(float);
    if (bytes % (2 * sizeof(float)) != 0) {
        std::cerr << "Warning: Buffer size does not match expected pair structure." << std::endl;
        return; // Return an empty vector if buffer size is incorrect
    }
    for (size_t i = 0; i < bytes; i += 2 * sizeof(float)) {
        float first, second;

        std::memcpy(&first, buffer.data() + i, sizeof(float)); // Extract first float
        std::memcpy(&second, buffer.data() + i + sizeof(float), sizeof(float)); // Extract second float

        frame.points.emplace_back(first, second); // Add pair to vector
    }
}
