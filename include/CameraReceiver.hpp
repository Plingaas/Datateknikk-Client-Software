//
// Created by Pling on 23/11/2024.
//

#ifndef CAMERARECEIVER_HPP
#define CAMERARECEIVER_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/cvconfig.h>
#include <simple_socket/TCPSocket.hpp>
#include <thread>
#include "simple_socket/UDPSocket.hpp"

class CameraReceiver {
public:
    CameraReceiver(const std::string& ip, uint16_t port);
    void record(std::string& filename, int fps);
    ~CameraReceiver() {
        if (recording) writer->release();
    };
private:
    std::unique_ptr<simple_socket::UDPSocket> client;
    cv::Mat frame;
    std::string serverIP;
    uint16_t serverPort;
    cv::Size frameSize;
    bool recording;

    std::unique_ptr<cv::VideoWriter> writer;
    void receive();


};
#endif //CAMERARECEIVER_HPP
