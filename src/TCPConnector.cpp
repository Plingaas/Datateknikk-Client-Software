//
// Created by Pling on 03/12/2024.
//

#include "TCPConnector.hpp"

TCPConnector::TCPConnector(std::string& ip_, uint16_t port_, bool verbose_) : TCPClientContext() {
    ip = ip_;
    port = port_;
    verbose = verbose_;
    connected = false;
    timeout = 0;
};

int TCPConnector::read(std::vector<uint8_t>& buffer, size_t bytes) {
    if (connected) {
        int bytesRead = connection->read(buffer.data(), bytes);
        if (bytesRead == -1) return -1;
        //if (verbose) printMessage("Received data.");
        last_read_time = getCurrentTime();
        return bytesRead;
    }

    if (connected) last_read_time = getCurrentTime();
    connected = false;

    if (getCurrentTime() - last_read_time < 1000) return -1;

    if (verbose) printMessage("Lost connection to server.");
    if (connection_lost_handler) connection_lost_handler();
    connected = false;

    std::thread disconnect_timeout_thread([this] {
        long time_of_disconnect = getCurrentTime();

        while (!connected) {
            if (getCurrentTime() > time_of_disconnect + timeout) {
                if (verbose) printMessage("Disconnected from server.");
                if (disconnect_handler) disconnect_handler();
                return;
            }
            connection = TCPClientContext::connect(ip, port);
        }
        if (verbose) printMessage("Restored connection to server.");
        if (connection_restored_handler) connection_restored_handler();
    });

    disconnect_timeout_thread.join();
    return -1;
}

int TCPConnector::read(std::vector<uint8_t>& buffer) {
    if (connected) {
        int bytesRead = 0;
        bool frameComplete;
        while (!frameComplete) {

            std::vector<uint8_t> tempBuffer(16384);

            bytesRead += connection->read(tempBuffer);
            if (bytesRead == -1) return -1;
            if (verbose) printMessage("Received data.");

            buffer.insert(buffer.end(), tempBuffer.begin(), tempBuffer.begin() + bytesRead);

            last_read_time = getCurrentTime();
        }
        return bytesRead;
    }

    if (connected) last_read_time = getCurrentTime();
    connected = false;

    if (getCurrentTime() - last_read_time < 1000) return -1;

    if (verbose) printMessage("Lost connection to server.");
    if (connection_lost_handler) connection_lost_handler();
    connected = false;

    std::thread disconnect_timeout_thread([this] {
        long time_of_disconnect = getCurrentTime();

        while (!connected) {
            if (getCurrentTime() > time_of_disconnect + timeout) {
                if (verbose) printMessage("Disconnected from server.");
                if (disconnect_handler) disconnect_handler();
                return;
            }
            connection = TCPClientContext::connect(ip, port);
        }
        if (verbose) printMessage("Restored connection to server.");
        if (connection_restored_handler) connection_restored_handler();
    });

    disconnect_timeout_thread.join();
    return -1;
}

void TCPConnector::connect() {
    connection = TCPClientContext::connect(ip, port);
    if (!connection) {
        if (verbose) printMessage("Failed to connect to server.");
        if (failed_to_connect_handler) failed_to_connect_handler();
        connected = false;
        return;
    }

    if (verbose) printMessage("Connected to server.");
    if (connect_handler) connect_handler();
    connected = true;
}

long TCPConnector::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return duration.count();
}

void TCPConnector::flush() {
    std::vector<uint8_t> flush_vector;
    connection->read(flush_vector);
}
