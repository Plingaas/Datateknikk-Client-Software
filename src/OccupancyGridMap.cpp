//
// Created by Pling on 03/12/2024.
//
#include "GridMap.hpp"

#include <vector>
#include <fstream>
#include <iostream>
#include <string>

GridMap::GridMap(int size_, int worldSize_) {
    gridSize = size_;
    worldSize = worldSize_;

    map = std::vector<std::vector<Cell>>(gridSize, std::vector<Cell>(gridSize));
    mapGraphics = cv::Mat(gridSize, gridSize, CV_8UC3, cv::Vec3b(128, 128, 128));
    framesSinceFrontier = 0;

    window.init("SLAM");

    cv::setMouseCallback(window.name, [](int event, int x, int y, int flags, void* userdata) {
        GridMap* gridMap = static_cast<GridMap*>(userdata);  // Cast userdata back to GridMap pointer

        if (event == cv::EVENT_LBUTTONDOWN) {
            gridMap->window.isDrawing = true;
            gridMap->setPixelToObstacle(x, y);
            gridMap->window.lastx = x;
            gridMap->window.lasty = y;
        }
        else if (event == cv::EVENT_MOUSEMOVE && gridMap->window.isDrawing) {
            if (!gridMap->isValidPosition(x,y)) return;
            auto cells = gridMap->bresenham(gridMap->window.lastx, gridMap->window.lasty, x, y, 0);
            for (auto& cell: cells) {
                gridMap->setPixelToObstacle(cell.x, cell.y);
                gridMap->window.lastx = cell.x;
                gridMap->window.lasty = cell.y;
            }
        }
        else if (event == cv::EVENT_LBUTTONUP) {
            gridMap->window.isDrawing = false;
        }
    }, this);
}

uint32_t GridMap::getGridIndex(float value) {
    return static_cast<uint32_t>(value / (worldSize / (static_cast<float>(gridSize)) )) + gridSize/2;
}


void GridMap::markLidarPoints(Frame& frame) {

    for (auto point : frame.lidarPoints) {
        int x = getGridIndex(point.x);
        int y = getGridIndex(point.y);
        if (!isValidPosition(x, y)) {
            continue;
        };
        map[y][x].obstacleCount++;
        if (map[y][x].obstacleCount > obstacleThreshold) {
            if (map[y][x].state != 2) {
                map[y][x].state = 2;
                int roverX = getGridIndex(frame.x);
                int roverY = getGridIndex(frame.y);
                auto cells = bresenham(roverX, roverY, x, y, 2);

                for (auto cell : cells) {
                    // Logikk for å se ratio mellom occupied og unoccupied
                    if (isValidPosition(cell) && map[cell.y][cell.x].state == 0) {
                        map[cell.y][cell.x].state = 1;
                    }
                }
            }
        }
    }
}

void GridMap::setFrame() {
    for (int y = 0; y < gridSize; y++) {
        int _y = gridSize - y - 1;
        for (int x = 0; x < gridSize; x++) {
            if (map[y][x].state == 0) {
                mapGraphics.at<cv::Vec3b>(_y, x) = cv::Vec3b(128, 128, 128);
            } else if (map[y][x].state == 1) {
                if (map[y][x].frontier) mapGraphics.at<cv::Vec3b>(_y, x) = cv::Vec3b(0, 0, 255);
                else mapGraphics.at<cv::Vec3b>(_y, x) = cv::Vec3b(255, 255, 255);
            } else if (map[y][x].state == 2) {
                mapGraphics.at<cv::Vec3b>(_y, x) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    if (!path) return;
    for (auto &cell : path) {
        if (isValidPosition(cell)) mapGraphics.at<cv::Vec3b>(gridSize-1-cell.y, cell.x) = cv::Vec3b(0, 255, 0);
    }
}

void GridMap::update(Frame& frame) {
    markLidarPoints(frame);
    framesSinceFrontier++;
    if (framesSinceFrontier > 5) {
        clearFrontiers();
        //markFrontiers(frame);
        uint32_t roverX = getGridIndex(frame.x);
        uint32_t roverY = getGridIndex(frame.y);

        auto centers = findLargestFrontierCenters();
        if (!centers.empty()) {
            float minDistance = 99999;
            for (auto& center : centers) {
                float distance = std::sqrt(std::pow(center.x - roverX,2) + std::pow(center.y - roverY,2));
                if (distance < minDistance) {
                    minDistance = distance;
                    centerFrontier = center;
                }
            }
        }

        Cell roverPos(roverX, roverY);
        path = AStar(roverPos, centerFrontier);

        /*for (auto& center : centers) {
            centerFrontier = center;
            if (!path.empty()) break; // Break as soon as we find a path.
        }*/


        int seeforward = 7; // How many tiles to see forward for driving the rover.
        if (path) {
            auto index = std::min(static_cast<int>(path.size()-1), seeforward);
            if (newPathHandler) newPathHandler(roverPos, path[index]);
        } else {
            saveGridToFile(map, "data/map.txt");
        }

        framesSinceFrontier = 0;
    }

    setFrame();
    cv::circle(mapGraphics, {centerFrontier.x, gridSize-1-centerFrontier.y}, 3, cv::Vec3b(255, 0, 0));

    cv::Mat copy;
    try {
        //cv::resize(mapGraphics, copy, cv::Size(1000, 1000));
        displayFrame();
    } catch (cv::Exception& e) {
        std::cout << e.what() << std::endl;
    }
}

void GridMap::displayFrame() {
    if (writer && recording) writer->write(mapGraphics);
    cv::imshow("SLAM", mapGraphics);
    cv::waitKey(1);
}
bool GridMap::obstacleOnBresenham(int x0, int y0, int x1, int y1) {
    std::vector<Cell> cells;
    std::set<std::pair<int, int>> visited;

    // Used for line thickness.
    bool over = y1 < y0;
    bool under = !over;
    bool right = x1 > x0;
    bool left = !right;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1.0 : -1.0;
    int sy = (y0 < y1) ? 1.0 : -1.0;
    int err = dx - dy;

    while (true) {
        if (map[y0][x0].state == 2) return true;
        cells.emplace_back(x0, y0);
        visited.insert(std::make_pair(y0, x0));
        // Check if the endpoint has been reached
        if (std::abs(x0 - x1) == 0 && std::abs(y0 - y1) == 0) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return false;
}
std::vector<Cell> GridMap::bresenham(int x0, int y0, int x1, int y1, int r) {
    std::vector<Cell> cells;
    std::set<std::pair<int, int>> visited;

    // Used for line thickness.
    int posx = x0;
    int posy = y0;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1.0 : -1.0;
    int sy = (y0 < y1) ? 1.0 : -1.0;
    int err = dx - dy;

    while (true) {
        if (!isValidPosition(x0, y0));
        cells.emplace_back(x0, y0);
        visited.insert(std::make_pair(y0, x0));
        // Check if the endpoint has been reached
        if (std::abs(x0 - x1) == 0 && std::abs(y0 - y1) == 0) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    // Improve readability of code

    bool over = y1 < posy;
    bool under = !over;
    bool right = x1 > posx;
    bool left = !right;

    std::set<std::pair<int, int>> explored;
    for (auto cell : visited) {
        for (int y = cell.first - r; y <= cell.first + r; y++) {
            if (over && y <= y1 || under && y >= y1) continue;
            for (int x = cell.second - r; x <= cell.second + r; x++) {
                if (left && x <= x1 || right && x >= x1) continue;
                if (visited.contains({y,x})) continue;
                if (obstacleOnBresenham(posx, posy, x, y)) continue;
                if (explored.contains({y, x})) continue;

                if (isValidPosition(x, y)) {
                    explored.insert(std::make_pair(y, x));
                    if (map[y][x].state == 0) {
                        cells.emplace_back(x, y);
                    }
                }
            }
        }
    }
    return cells;
}

void GridMap::markFrontiers(Frame& frame) {
    std::queue<Cell> q;
    std::set<std::pair<int, int>> visited;

    int roverX = getGridIndex(frame.x);
    int roverY = getGridIndex(frame.y);

    Cell start(roverX, roverY);
    q.push(start);
    visited.insert({roverY, roverX});

    // Directions for neighbors (up, down, left, right)
    Cell W(-1, 0);
    Cell NW(-1, 1);
    Cell N (0, 1);
    Cell NE(1, 1);
    Cell E(1, 0);
    Cell SE(1, -1);
    Cell S(0, -1);
    Cell SW(-1, -1);

    const std::vector<Cell> directions = {
        N, NE, E, SE, S, SW, W, NW
    };

    while (!q.empty()) {
        auto currentCell = q.front();
        q.pop();

        // If the current cell is a frontier, add it to the result
        if (isFrontier(currentCell)) {
            map[currentCell.y][currentCell.x].frontier = true;
        }

        // Explore neighbors
        for (const auto& dCell : directions) {
            auto neighbour = currentCell + dCell;

            if (isValidPosition(neighbour) && visited.find({neighbour.y, neighbour.x}) == visited.end()) {
                visited.insert({neighbour.y, neighbour.x});
                if (map[neighbour.y][neighbour.x].state == 1) {
                    q.push(neighbour);
                }
            }
        }
    }
}

std::vector<Cell> GridMap::findLargestFrontierCenters() {
    std::queue<Cell> q;
    std::set<std::pair<int, int>> visited;
    std::vector<std::vector<Cell>> frontiers;
    std::vector<std::vector<Cell>> frontiersExcluded;

    // Directions for neighbors (up, down, left, right, diagonals)
    Cell W(-1, 0);
    Cell NW(-1, 1);
    Cell N(0, 1);
    Cell NE(1, 1);
    Cell E(1, 0);
    Cell SE(1, -1);
    Cell S(0, -1);
    Cell SW(-1, -1);

    const std::vector<Cell> directions = {N, NE, E, SE, S, SW, W, NW};

    for (int row = 0; row < gridSize; row++) {
        for (int col = 0; col < gridSize; col++) {
            Cell start(row, col);
            std::pair<int, int> curr = {row, col};
            if (isFrontier(start) && visited.find(curr) == visited.end()) {
                // Perform BFS to collect all connected frontier points
                std::vector<Cell> frontier;
                q.push(start);
                visited.insert(curr);

                while (!q.empty()) {
                    Cell currentCell = q.front();
                    q.pop();
                    frontier.push_back(currentCell);

                    for (const auto& dCell : directions) {
                        Cell neighbor = currentCell - dCell;

                        if (isValidPosition(neighbor) &&
                            isFrontier(neighbor) &&
                            visited.find({neighbor.y, neighbor.x}) == visited.end()) {

                            visited.insert({neighbor.y, neighbor.x});
                            q.push(neighbor);
                        }
                    }
                }
                if (frontier.size() > 30 && frontier.size() < 300) frontiers.push_back(frontier);
                else frontiersExcluded.push_back(frontier);
            }
        }
    }

    // Sort the frontiers by size in descending order
    std::sort(frontiers.begin(), frontiers.end(), [](const auto& a, const auto& b) {
        return a.size() > b.size();
    });
    std::sort(frontiersExcluded.begin(), frontiersExcluded.end(), [](const auto& a, const auto& b) {
            return a.size() > b.size();
    });

    // Find the middle points
    std::vector<Cell> centers;
    int frontierCount = 5;
    if (frontiers.size() == 0) frontiers = frontiersExcluded;
    for (int i = 0; i < std::min(frontierCount, static_cast<int>(frontiers.size())); ++i) {
        const auto& frontier = frontiers[i];
        centers.push_back(frontier[frontier.size() / 2]); // Middle point
    }

    // If we didnt find enough frontier in the main frontiers, start adding from excluded frontiers
    if (centers.size() < frontierCount) {
        for (int i = 0; i < std::min(frontierCount-centers.size(), frontiersExcluded.size()); i++) {
            const auto& frontier = frontiersExcluded[i];
            centers.push_back(frontier[frontier.size() / 2]); // Middle point
        }
    }

    // Set all remaining frontier's cells
    for (const auto& frontier : frontiers) {
        for (const auto& cell : frontier) {
            map[cell.y][cell.x].frontier = true;
        }
    }

    return centers;
}


bool GridMap::isFrontier(Cell& cell) {
    int row = cell.y;
    int col = cell.x;
    if (map[row][col].state != 1 ) return false; // Must be a '0'

    Cell W(-1, 0);
    Cell NW(-1, 1);
    Cell N (0, 1);
    Cell NE(1, 1);
    Cell E(1, 0);
    Cell SE(1, -1);
    Cell S(0, -1);
    Cell SW(-1, -1);

    const std::vector<Cell> directions = {
        N, NE, E, SE, S, SW, W, NW
    };

    for (const auto dCell : directions) {
        auto neighbour = cell + dCell;
        if (isValidPosition(neighbour) && map[neighbour.y][neighbour.x].state == 0) {
            return true;
        }
    }
    return false;
}

bool GridMap::isValidPosition(Cell& cell) {
    return cell.y >= 0 && cell.y < gridSize && cell.x >= 0 && cell.x < gridSize;
}

bool GridMap::isValidPosition(int x, int y) {
    return y >= 0 && y < gridSize && x >= 0 && x < gridSize;
}

void GridMap::clearFrontiers() {
     for (auto& row : map) {
         for (auto& cell : row) {
             cell.frontier = false;
         }
     }
}

void GridMap::saveGridToFile(const std::vector<std::vector<Cell>>& grid, const std::string& filename) {
    std::ofstream outFile(filename);

    if (!outFile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Write header: width of the grid
    outFile << "Gridsize:" << grid.size() << "\n";

    // Iterate through the 2D vector and save each Cell's state
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            outFile << cell.state << " ";
        }
        outFile << "\n"; // Newline after each row
    }

    outFile.close();
    std::cout << "Grid saved to " << filename << std::endl;
}

void GridMap::screenRecord(std::string& filename, int fps) {

    writer = std::make_unique<cv::VideoWriter>(
        "videos/" + filename,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        cv::Size(gridSize, gridSize)
    );

    recording = writer->isOpened();
}

void GridMap::setPixelToObstacle(int x, int y) {
    int _y = gridSize - y - 1;

    map[_y][x].obstacleCount = 99999;
    map[_y][x].state = 2;
}