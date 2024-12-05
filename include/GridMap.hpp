//
// Created by Pling on 03/12/2024.
//

#ifndef GRIDMAP_H
#define GRIDMAP_H
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "DataTypes.hpp"

class GridMap {

public:

    using NewPathHandler = std::function<void(Cell& current, Cell& target)>;
    void setNewPathHandler (NewPathHandler handler) {newPathHandler = std::move(handler);};
    GridMap(int size_, int worldSize_);
    void setObstacleThreshold(const int obstacleThreshold_) {obstacleThreshold = obstacleThreshold_;};
    void update(Frame& frame);


    std::vector<Cell> AStar(const Cell &start, const Cell &goal);
private:

    NewPathHandler newPathHandler;

    void markLidarPoints(Frame& frame);
    void setFrame();
    int getGridIndex(float value);

    void markFrontiers(Frame& frame);
    void clearFrontiers();
    std::vector<Cell> findLargestFrontierCenters();
    std::vector<Cell> bresenham(int x0, int y0, int x1, int y1, int r = 1);
    bool obstacleOnBresenham(int x0, int y0, int x1, int y1);
    bool isFrontier(Cell& cell);
    bool isValidPosition(Cell& cell);
    Cell centerFrontier;
    std::vector<Cell> path;

    std::vector<uint8_t> brightness = {0, 128, 255};
    int obstacleThreshold;
    int gridSize;
    int worldSize;
    std::vector<std::vector<Cell>> map;
    cv::Mat mapGraphics;

    int framesSinceFrontier;
};
#endif //GRIDMAP_H
