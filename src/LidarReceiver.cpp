//
// Created by Pling on 21/11/2024.
//

#include "LidarReceiver.hpp"

LidarReceiver::LidarReceiver(const std::string& _ip, uint16_t _port) {
    ip = _ip;
    port = _port;
    client = std::make_unique<TCPClientContext>();
    logger = std::make_unique<CSVWriter>("data/LidarData.csv", "time,points");

    receiverThread = std::thread([this, _ip, _port] {
        try {
            auto connection = client->connect(_ip, _port);
            std::cout << "Connected to lidar server." << std::endl;
            int dataCount = 0;
            while (true) {
                std::vector<unsigned char> buffer(8192);
                connection->read(buffer);
                auto now = std::chrono::system_clock::now();
                auto msSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

                auto data = deserializeLidarData(buffer);

                std::vector<float> points;
                if (data[0].first == 0 && data[1].first == 0) {continue;};

                for (auto pair : data) {
                    points.push_back(static_cast<float>(pair.first));
                    points.push_back(static_cast<float>(pair.second));
                }

                logger->log(msSinceEpoch, points);
                if (dataCount % 10 == 0) {
                   std::cout <<  "Lidar data line count: " << dataCount++ << std::endl;
               }
            }
        } catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    });

    receiverThread.detach();
}

std::vector<std::pair<double, double> > LidarReceiver::deserializeLidarData(const std::vector<unsigned char> &buffer) {
    std::vector<std::pair<double, double>> data;
    size_t numBytes = buffer.size();

    if (numBytes % (2 * sizeof(double)) != 0) {
        std::cerr << "Warning: Buffer size does not match expected pair structure." << std::endl;
        return data; // Return an empty vector if buffer size is incorrect
    }

    for (size_t i = 0; i < numBytes; i += 2 * sizeof(double)) {
        double first, second;

        std::memcpy(&first, buffer.data() + i, sizeof(double)); // Extract first double
        std::memcpy(&second, buffer.data() + i + sizeof(double), sizeof(double)); // Extract second double

        data.emplace_back(first, second); // Add pair to vector
    }

    data.erase(
    std::remove(data.begin(), data.end(), std::make_pair(720.0, 720.0)),
    data.end()
    );

    return data;
}
