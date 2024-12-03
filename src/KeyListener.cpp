//
// Created by Pling on 02/12/2024.
//

#include "KeyListener.hpp"

#include <iostream>
#include <ostream>

KeyListener::KeyListener() {

    std::thread listen_thread([this] {
        while (true) {
            if (_kbhit()) {
                int key = _getch();

                if (key == 224) { // Keyprefix
                    key = _getch();
                }
                keyStates.insert(key);
            }

            for (int key : keyStates) {
                bool pressed = !lastKeyStates.contains(key);
                if (pressed) onKeyPressed(key);

                bool held = lastKeyStates.contains(key);
                if (held) onKeyHeld(key);
            }

            for (int key : lastKeyStates) {
                bool released = !keyStates.contains(key);
                //if (released) onKeyReleased(key);
            }

            lastKeyStates = keyStates;
            keyStates.clear();
        }
    });

    listen_thread.detach();
};