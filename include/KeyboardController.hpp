//
// Created by Pling on 02/12/2024.
//

#ifndef KEYBOARDCOMMANDSTREAMER_HPP
#define KEYBOARDCOMMANDSTREAMER_HPP

#include "CommandStreamer.hpp"

class KeyboardController : public CommandStreamer {

public:
    explicit KeyboardController(uint16_t TCPPort = 5853);
    void onKeyPressed(int key);
    void onKeyHeld(int key);
    void onKeyReleased(int key);

private:
    std::unique_ptr<KeyListener> keyListener;

};

#endif //KEYBOARDCOMMANDSTREAMER_HPP
