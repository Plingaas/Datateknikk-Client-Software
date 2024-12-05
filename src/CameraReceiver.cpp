//
// Created by Pling on 28/11/2024.
//

#include "CameraReceiver.hpp"
#include <iomanip>

#define BUFFER_SIZE 1024

CameraReceiver::CameraReceiver(const std::string& ip, uint16_t port) {
#ifdef HAS_CUDA
    std::cout << "OpenCV found CUDA support."
#else
    std::cout << "CUDA NOT SUPPORTED" << std::endl;
#endif

    serverIP = ip;
    serverPort = port;
    frameSize = cv::Size(416, 416);
    recording = false;

    std::thread receiver_thread([this] {
        client = std::make_unique<simple_socket::UDPSocket>(8553);
        receive();
    });

    receiver_thread.detach();
};


void CameraReceiver::receive() {

    std::vector<uint8_t> SOF(4);
    while (true) {
        int bytes = client->recvFrom(serverIP, serverPort, SOF);
        if (SOF[0] != 0xd8 || SOF[3] != 0x8d || SOF.size() != 4) {
            //std::cout << "Wrong SOF. Received " << bytes << " bytes." << std::endl;
            continue;
        }

        uint16_t nPackets = SOF[1] << 8 | SOF[2];
        std::vector<uint8_t> img;
        std::vector<std::vector<uint8_t>> packets(nPackets, std::vector<uint8_t>(BUFFER_SIZE));
        uint32_t bytesReceived = 0;
        for (int i = 0; i < nPackets; i++) {
            bytesReceived += client->recvFrom(serverIP, serverPort, packets[i]);
        }

        for (int i = 0; i < nPackets; i++) {
            //std::cout << "id: " << static_cast<int>(packets[i][0]) << std::endl;
            img.insert(img.end(), packets[i].begin() + 1, packets[i].end());
        }

        try {
            auto newimg = cv::imdecode(img, cv::IMREAD_COLOR);

            cv::resize(newimg, newimg, frameSize, cv::INTER_CUBIC);
            cv::imshow("img", newimg);
            if (recording) writer->write(newimg);
            cv::waitKey(1);
        } catch (cv::Exception e) {
            std::cout << e.what() << std::endl;
        }
    }
}

void CameraReceiver::record(std::string& filename, int fps) {
    writer = std::make_unique<cv::VideoWriter>(
        "videos/" + filename,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        frameSize
    );

    recording = writer->isOpened();
}