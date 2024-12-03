//
// Created by Pling on 02/12/2024.
//

#include "CommandStreamer.hpp"

CommandStreamer::CommandStreamer(uint16_t TCPPort) {
    streamer = std::make_unique<TCPStreamer>(TCPPort, true);
    streamer->setName("CommandStreamer");
    streamer->setConnectHandler([this] {onClientConnect();});
    streamer->setDisconnectHandler([this] {onClientDisconnect();});
    streamer->startStreaming();
}

void CommandStreamer::onClientConnect() {
    std::cout << "(CommandStreamer): " << "Client connected." << std::endl;
}

void CommandStreamer::onClientDisconnect() {
    streamer->clearPackets();
}

void CommandStreamer::addCommand(int cmd, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> command = {static_cast<uint8_t>(cmd)};

    switch (cmd) {
        case Command::FORWARD:
            break;
        case Command::BACKWARD:
            break;
        case Command::TURN_LEFT:
            break;
        case Command::TURN_RIGHT:
            break;
        case Command::STOP:
            break;
        case Command::STOP_TURNING:
            break;
        case Command::SET_HEADING:
            command.insert(command.end(), params.begin(), params.end());
            break;
        case Command::SET_SPEED:
            command.insert(command.end(), params.begin(), params.end());
            break;
        case Command::SET_TURN_SPEED:
            command.insert(command.end(), params.begin(), params.end());
            break;
        default:
            std::cout << "Unknown command code: " << cmd << std::endl;
            return;
    }

    streamer->addPacket(command);
}

