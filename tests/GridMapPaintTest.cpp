//
// Created by Pling on 05/12/2024.
//

#include "GridMap.hpp"

int main() {
    int mapSize = 100;
    int gridSize = mapSize * 10;
    int obstacleThreshold = 28;

    GridMap map(gridSize, mapSize);
    map.setObstacleThreshold(obstacleThreshold);

    while (true) {
        map.setFrame();
        map.displayFrame();
    }
}