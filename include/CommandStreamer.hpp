//
// Created by Pling on 02/12/2024.
//

#ifndef COMMANDSTREAMER_HPP
#define COMMANDSTREAMER_HPP
#include "KeyListener.hpp"
#include "TCPStreamer.hpp"
#include "CommandsEnum.hpp"

class CommandStreamer {
public:
    explicit CommandStreamer(uint16_t TCPPort);

    void onClientConnect();
    void onClientDisconnect();

    void addCommand(int cmd, const std::vector<uint8_t>& params = {});
private:
    std::unique_ptr<TCPStreamer> streamer;
};
#endif //COMMANDSTREAMER_HPP
