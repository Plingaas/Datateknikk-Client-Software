//
// Created by Pling on 02/12/2024.
//
#include "KeyboardController.hpp"

KeyboardController::KeyboardController(uint16_t TCPPort) : CommandStreamer(TCPPort) {
    keyListener = std::make_unique<KeyListener>();
    keyListener->setOnKeyPressed([this](int key) {onKeyPressed(key);});
    keyListener->setOnKeyHeld([this](int key) {onKeyHeld(key);});
    keyListener->setOnKeyReleased([this](int key) {onKeyReleased(key);});
};

void KeyboardController::onKeyPressed(int key) {
    std::cout << "Keypress: "<< key << std::endl;
    switch(key) {
        case Key::UP:
            addCommand(Command::FORWARD);
        break;

        case Key::DOWN:
            addCommand(Command::BACKWARD);
        break;

        case Key::LEFT:
            addCommand(Command::TURN_LEFT);
            break;

        case Key::RIGHT:
            addCommand(Command::TURN_RIGHT);
            break;

        default:
            break;
    }
};

void KeyboardController::onKeyHeld(int key) {
    std::cout << "Keyheld: "<< key << std::endl;
}

void KeyboardController::onKeyReleased(int key) {
    std::cout << "Keyreleased: "<< key << std::endl;
    switch(key) {
        case Key::UP:
            addCommand(Command::STOP);
        break;

        case Key::DOWN:
            addCommand(Command::STOP);
        break;

        case Key::LEFT:
            addCommand(Command::STOP_TURNING);
        break;

        case Key::RIGHT:
            addCommand(Command::STOP_TURNING);
        break;

        default:
            break;
    }
}