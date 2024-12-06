//
// Created by Pling on 03/12/2024.
//

#ifndef GRIDMAP_H
#define GRIDMAP_H
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "DataTypes.hpp"
#include "AStar.hpp"

class OccupancyGridMap {

public:

    using NewPathHandler = std::function<void(Cell& current, Cell& target)>;
    void setNewPathHandler (NewPathHandler handler) {newPathHandler = std::move(handler);};
    OccupancyGridMap(int size_, int worldSize_);
    void setObstacleThreshold(const int obstacleThreshold_) {obstacleThreshold = obstacleThreshold_;};
    void update(Frame& frame);
    void screenRecord(std::string& filename, int fps);
    void setFrame();
    void displayFrame();
    ~OccupancyGridMap() {
        if (recording) writer->release();
    }
private:
    NewPathHandler newPathHandler;
    AStar pathfinder;

    void setPixelToObstacle(int x, int y);
    void markLidarPoints(Frame& frame);
    uint32_t getGridIndex(float value);

    void markFrontiers(Frame& frame);
    void clearFrontiers();
    std::vector<Cell> findLargestFrontierCenters();
    std::vector<Cell> bresenham(int x0, int y0, int x1, int y1, int r = 1);
    bool obstacleOnBresenham(int x0, int y0, int x1, int y1);
    bool isFrontier(Cell& cell);
    bool isValidPosition(Cell& cell);
    bool isValidPosition(int x, int y);
    Cell centerFrontier;
    Path path;
    PaintWindow window;
    int obstacleThreshold;
    int gridSize;
    int worldSize;
    std::vector<std::vector<Cell>> map;
    cv::Mat mapGraphics;
    int framesSinceFrontier;
    void saveGridToFile(const std::vector<std::vector<Cell>>& grid, const std::string& filename);
    std::unique_ptr<cv::VideoWriter> writer;
    bool recording;
};
#endif //GRIDMAP_H
