//
// Created by Pling on 02/12/2024.
//

#ifndef KEYLISTENER_HPP
#define KEYLISTENER_HPP

#include <conio.h>
#include <functional>
#include <thread>
#include <set>
#include "KeysEnum.hpp"

class KeyListener {

public:
    KeyListener();

    using OnKeyPressed = std::function<void(int keyCode)>;
    using OnKeyReleased = std::function<void(int keyCode)>;
    using OnKeyHeld = std::function<void(int keyCode)>;

    void setOnKeyPressed(OnKeyPressed onKeyPressed_) {
        onKeyPressed = std::move(onKeyPressed_);
    };

    void setOnKeyHeld(OnKeyPressed onKeyHeld_) {
        onKeyHeld = std::move(onKeyHeld_);
    }

    void setOnKeyReleased(OnKeyReleased onKeyReleased_) {
        onKeyReleased = std::move(onKeyReleased_);
    };


private:
  OnKeyPressed onKeyPressed;
  OnKeyReleased onKeyReleased;
  OnKeyHeld onKeyHeld;

  std::set<int> keyStates;
  std::set<int> lastKeyStates;
};
#endif //KEYLISTENER_HPP
