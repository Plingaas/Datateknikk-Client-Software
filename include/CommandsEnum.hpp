//
// Created by Pling on 02/12/2024.
//

#ifndef DRIVECOMMANDS_HPP
#define DRIVECOMMANDS_HPP

namespace Command {
  enum Drive {
    STOP = 0x00,
    FORWARD = 0x01,
    BACKWARD = 0x02,
    TURN_LEFT = 0x03,
    TURN_RIGHT = 0x04,
    STOP_TURNING = 0x05,
    DRIVE_TO_POSITION = 0x0f
  };

  enum Configure {
    SET_SPEED = 0xf0,
    SET_TURN_SPEED = 0xf1,
    SET_HEADING = 0xf2
  };
}
#endif //DRIVECOMMANDS_HPP
