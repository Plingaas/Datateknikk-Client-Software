#include "DataTypes.hpp"
#include "GridMap.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
int getGridIndex(double value, int gridSize, int worldSize) {
    return static_cast<int>(value / (worldSize / (static_cast<float>(gridSize)) ));
}

std::vector<std::pair<RoverFrame, LidarFrame>> readDataset(const std::string& filename, int numLines, int gridSize, int mapSize) {
    std::cout << "Reading " << filename << " with " << numLines << " lines." << std::endl;
    std::vector<std::pair<RoverFrame, LidarFrame>> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Error: Could not open file " + filename);
    }

    std::string line;
    int lineCount = 0;
    while (std::getline(file, line) && lineCount < numLines) {
        lineCount++;

        // Split line by semicolon
        std::stringstream ss(line);
        std::string t, posXStr, posYStr, headingStr, lidarData;
        std::getline(ss, t, ';');
        std::getline(ss, posXStr, ';');
        std::getline(ss, posYStr, ';');
        std::getline(ss, headingStr, ';');
        std::getline(ss, lidarData);

        double posX = std::stod(posXStr);
        double posY = std::stod(posYStr);
        double heading = std::stod(headingStr);

        // Apply transformation for the position
        double _posX = posY;
        double _posY = -posX;

        RoverFrame roverFrame;
        roverFrame.yaw = heading;
        roverFrame.x = _posX;
        roverFrame.y = _posY;

        int roverGridX = getGridIndex(_posX, gridSize, mapSize);
        int roverGridY = getGridIndex(_posY, gridSize, mapSize);

        // Parse lidar data (assuming it's formatted as "x1,y1:x2,y2:...")
        LidarFrame lidarFrame;
        std::stringstream lidarStream(lidarData);
        std::string point;
        while (std::getline(lidarStream, point, ':')) {
            std::stringstream pointStream(point);
            double x, y;
            char comma; // To read the comma between x and y

            if (pointStream >> x >> comma >> y) {
                double _x = x * std::cos(heading) - y * std::sin(heading) + _posX;
                double _y = x * std::sin(heading) + y * std::cos(heading) + _posY;
                // Default confidence value set to 0
                lidarFrame.points.push_back({_x, _y});
            } else {
                std::cerr << "Failed to parse lidar point: " << point << std::endl;
            }
        }

        // Create and add frame to the vector
        std::pair<RoverFrame, LidarFrame> pair(roverFrame, lidarFrame);
        data.push_back(pair);
    }

    file.close();
    return data;
}

int main() {

    int gridSize = 500;
    int mapSize = 50;
    int obstacleThreshold = 20;
    const std::string filepath = "data/ProcessedData1.txt";
    int numLinesToRead = 1500;
    auto frames = readDataset(filepath, numLinesToRead, gridSize, mapSize);

    GridMap gridmap(gridSize, mapSize);
    gridmap.setObstacleThreshold(obstacleThreshold);
    int i = 0;
    for (auto& frame : frames) {
        Frame processedFrame(frame.first, frame.second, false);
        gridmap.update(processedFrame);
        std::cout << i++ << std::endl;
    }

}