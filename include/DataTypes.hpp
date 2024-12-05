//
// Created by Pling on 03/12/2024.
//

#ifndef DATATYPES_HPP
#define DATATYPES_HPP
#include <vector>
#include <iostream>

struct LidarFrame {
    std::vector<std::pair<double, double>> points;
};

struct RoverFrame {
    float pitch, roll, yaw = 0;
    float ax, ay, az = 0;
    float gx, gy, gz = 0;
    float x, y = 0;
    float vx, vy = 0;

    RoverFrame() {};

    RoverFrame(const float* data) {
        pitch = data[0];
        roll = data[1];
        yaw = data[2];
        ax = data[3];
        ay = data[4];
        az = data[5];
        gx = data[6];
        gy = data[7];
        gz = data[8];
        x = data[9];
        y = data[10];
        vx = data[11];
        vy = data[12];
    };

    RoverFrame copy() {
        return RoverFrame(getData().data());
    }

    std::vector<float> getData() {
        return {pitch, roll, yaw, ax, ay, az, gx, gy, gz, x, y, vx, vy};
    }

    void print() {
        std::cout << "Pitch: " << pitch << " | Roll: " << roll << " | Yaw: " << yaw << std::endl;
        std::cout << "Ax: " << ax << " | AY: " << ay << " | AZ: " << az << std::endl;
        std::cout << "GX: " << gx << " | GY: " << gy << std::endl;
        std::cout << "x: " << x << " | y: " << y << std::endl;
        std::cout << "vx: " << vx << " | vy: " << vy << std::endl << std::endl;
    }
};

struct LidarPoint {
    float x;
    float y;
};

struct Cell {
    int x;
    int y;
    int obstacleCount;
    int notObstacleCount;
    int state;
    bool frontier = false;

    Cell(int x_ = 0, int y_ = 0, int obstacleCount_ = 0, int notObstacleCount_ = 0, int state_ = 0)
      : x(x_), y(y_), obstacleCount(obstacleCount_), notObstacleCount(notObstacleCount_), state(state_) {}

    Cell operator-(const Cell& other) {
        return Cell(x - other.x, y - other.y, obstacleCount, notObstacleCount, state);
    }

    Cell operator+(const Cell& other) {
        return Cell(x + other.x, y + other.y, obstacleCount, notObstacleCount, state);
    }

    bool operator==(const Cell& other) {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Cell& other) {
        return x != other.x || y != other.y;
    }

    void operator-=(const Cell& other) {
        x -= other.x;
        y -= other.y;
    }

    void operator+=(const Cell& other) {
        x += other.x;
        y += other.y;
    }

    bool operator<(const Cell &other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

struct Frame {
    float x;
    float y;
    float heading;
    std::vector<LidarPoint> lidarPoints;

    // Optional constructor
    Frame () {
        x = 0;
        y = 0;
        heading = 0;
        lidarPoints = {};
    }

    Frame (const RoverFrame& roverFrame, const LidarFrame& lidarFrame, bool process = true) {
        if (process) {
            x = roverFrame.y;
            y = -roverFrame.x;
            heading = roverFrame.yaw * 3.14159265358979323846 / 180.0f;

            for (auto p : lidarFrame.points) {
                LidarPoint point;
                point.x = p.first * std::cos(heading) - p.second * std::sin(heading) + x;
                point.y = p.first * std::sin(heading) + p.second * std::cos(heading) + y;
                lidarPoints.push_back(point);
            };
        } else {
            x = roverFrame.x;
            y = roverFrame.y;
            heading = roverFrame.yaw;
            for (auto p : lidarFrame.points) {
                LidarPoint point;
                point.x = p.first;
                point.y = p.second;
                lidarPoints.push_back(point);
            };
        }

    };
};

struct Node {
    Cell pos;
    double gCost; // Cost from start to current node
    double fCost; // Estimated total cost (gCost + heuristic)

    // Constructor
    Node(const Cell &p, double g, double f) : pos(p), gCost(g), fCost(f) {}

    bool operator>(const Node &other) const {
        return fCost > other.fCost;
    }
};
#endif //DATATYPES_HPP
