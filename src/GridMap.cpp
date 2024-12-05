//
// Created by Pling on 03/12/2024.
//
#include "GridMap.hpp"


GridMap::GridMap(int size_, int worldSize_) {
    gridSize = size_;
    worldSize = worldSize_;

    map = std::vector<std::vector<Cell>>(gridSize, std::vector<Cell>(gridSize));
    mapGraphics = cv::Mat(gridSize, gridSize, CV_8UC3, cv::Vec3b(128, 128, 128));
    framesSinceFrontier = 0;
}

int GridMap::getGridIndex(float value) {
    return static_cast<int>(value / (worldSize / (static_cast<float>(gridSize)) )) + gridSize/2;
}


void GridMap::markLidarPoints(Frame& frame) {

    for (auto point : frame.lidarPoints) {
        int x = getGridIndex(point.x);
        int y = getGridIndex(point.y);
        Cell current(x, y);
        if (!isValidPosition(current)) {
            continue;
        };
        map[y][x].obstacleCount++;
        if (map[y][x].obstacleCount > obstacleThreshold) {
            if (map[y][x].state != 2) {
                map[y][x].state = 2;
                int roverX = getGridIndex(frame.x);
                int roverY = getGridIndex(frame.y);
                auto cells = bresenham(roverX, roverY, x, y, 3);

                for (auto cell : cells) {
                    // Logikk for å se ratio mellom occupied og unoccupied
                    if (map[cell.y][cell.x].state == 0) {
                        map[cell.y][cell.x].state = 1;
                    }
                }
            }
        }
    }
}

void GridMap::setFrame() {
    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            if (map[y][x].state == 0) {
                mapGraphics.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
                //mapGraphics.at<uint8_t>(y, x) = 0;
            } else if (map[y][x].state == 1) {
                if (map[y][x].frontier) mapGraphics.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
                else mapGraphics.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                //mapGraphics.at<uint8_t>(y, x) = 0;
            } else if (map[y][x].state == 2) {
                mapGraphics.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                //mapGraphics.at<uint8_t>(y, x) = 0;
            }

                //mapGraphics.at<uint8_t>(y, x) = 255;
        }
    }

    for (auto &cell : path) {
        mapGraphics.at<cv::Vec3b>(cell.y, cell.x) = cv::Vec3b(0, 255, 0);
    }
}

void GridMap::update(Frame& frame) {
    markLidarPoints(frame);
    framesSinceFrontier++;
    std::vector<Cell> path;
    if (framesSinceFrontier > 10) {
        clearFrontiers();
        //markFrontiers(frame);
        int roverX = getGridIndex(frame.x);
        int roverY = getGridIndex(frame.y);

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

        int seeforward = 30;

        if (!path.empty()) {
            auto index = std::min(static_cast<int>(path.size()-1), seeforward);
            if (newPathHandler) newPathHandler(roverPos, path[index]);
            path = bresenham(roverX, roverY, path[index].x, path[index].y);
        }

        framesSinceFrontier = 0;
    }

    setFrame();
    cv::circle(mapGraphics, {centerFrontier.x, centerFrontier.y}, 3, cv::Vec3b(255, 0, 0));
    for (auto& cell : path) {
        mapGraphics.at<cv::Vec3b>(cell.y,cell.x) = cv::Vec3b(0, 255, 0);
    }
    cv::Mat copy;
    try {
        cv::resize(mapGraphics, copy, cv::Size(1000, 1000));
        // Show the updated frame
        cv::imshow("SLAM", copy);
        cv::waitKey(1);
    } catch (cv::Exception& e) {
        std::cout << e.what() << std::endl;
    }
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

    for (auto cell : visited) {
        for (int y = cell.first - r; y <= cell.first + r; y++) {
            if (over && y <= y1 || under && y >= y1) continue;
            for (int x = cell.second - r; x <= cell.second + r; x++) {
                if (left && x <= x1 || right && x >= x1) continue;
                if (visited.contains({y,x})) continue;
                if (obstacleOnBresenham(posx, posy, x, y)) continue;

                if (isValidPosition(map[y][x])) {
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
                if (frontier.size() > 50) frontiers.push_back(frontier);
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

    // Find the middle points of the three largest frontiers
    std::vector<Cell> centers;
    if (frontiers.size() == 0) frontiers = frontiersExcluded;
    for (int i = 0; i < std::min(1, static_cast<int>(frontiers.size())); ++i) {
        const auto& frontier = frontiers[i];
        centers.push_back(frontier[frontier.size() / 2]); // Middle point
    }

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

void GridMap::clearFrontiers() {
     for (auto& row : map) {
         for (auto& cell : row) {
             cell.frontier = false;
         }
     }
}

std::vector<Cell> GridMap::AStar(const Cell &start, const Cell &goal) {

    auto heuristic = [](const Cell &a, const Cell &b) {
        return std::abs(a.y - b.y) + std::abs(a.x - b.x);
    };

    auto calculateWallPenalty = [this](const Cell &pos) {
        double penalty = 0.0;
        for(int i=0; i < 10; ++i) {
            Cell W(-i, 0);
            Cell NW(-i, i);
            Cell N (0, i);
            Cell NE(i, i);
            Cell E(i, 0);
            Cell SE(i, -i);
            Cell S(0, -i);
            Cell SW(-i, -i);

            const std::vector<Cell> directions = {
                N, NE, E, SE, S, SW, W, NW
            };

            for (const auto &cell : directions) {
                int y = pos.y + cell.y;
                int x = pos.x + cell.x;
                Cell tile(x, y);
                if (isValidPosition(tile) && (map[y][x].state != 1 || map[y][x].frontier)) {
                    penalty += 10; // Add a penalty for proximity to walls
                }
            }
        }

        return penalty;
    };

    auto calculateDiagonalPenalty = [this](const Cell &pos) {
        double penalty = 0.0;
        for(int i=0; i < 10; ++i) {
            Cell W(-i, 0);
            Cell NW(-i, i);
            Cell N (0, i);
            Cell NE(i, i);
            Cell E(i, 0);
            Cell SE(i, -i);
            Cell S(0, -i);
            Cell SW(-i, -i);

            const std::vector<Cell> directions = {
                N, NE, E, SE, S, SW, W, NW
            };

            for (const auto &cell : directions) {
                int y = pos.y + cell.y;
                int x = pos.x + cell.x;
                Cell tile(x, y);
                if (isValidPosition(tile) && map[y][x].state != 1) {
                    penalty += 10; // Add a penalty for proximity to walls
                }
            }
        }

        return penalty;
    };


    using NodeQueue = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
    NodeQueue openSet;

    std::set<Cell> closedSet;
    std::map<Cell, Cell> cameFrom;
    std::map<Cell, double> gCost;

    gCost[start] = 0;
    openSet.emplace(start, 0, heuristic(start, goal)); // Use constructor here

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
    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        // Check if we reached a cell marked with 0


        if (current.pos == goal) {
            // Reconstruct the path
            std::vector<Cell> path;
            for (Cell p = goal; p != start; p = cameFrom[p]) {
                path.push_back(p);
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closedSet.insert(current.pos);

        for (const auto &cell : directions) {
            Cell neighbor = {current.pos.x + cell.x, current.pos.y + cell.y};

            if (!isValidPosition(neighbor) || map[neighbor.y][neighbor.x] == 2 || closedSet.count(neighbor)) {
                continue; // Skip invalid or blocked cells
            }

            double wallPenalty = calculateWallPenalty(neighbor);
            bool diagonalCell = (std::abs(current.pos.x-cell.x) == 1 && std::abs(current.pos.y-cell.y) == 1);
            double diagonalPenalty = diagonalCell * 3;

            double tentativeGCost = gCost[current.pos] + 1 + wallPenalty + diagonalPenalty; // Include wall penalty

            if (!gCost.count(neighbor) || tentativeGCost < gCost[neighbor]) {
                cameFrom[neighbor] = current.pos;
                gCost[neighbor] = tentativeGCost;
                double fCost = tentativeGCost + heuristic(neighbor, goal);

                openSet.emplace(neighbor, tentativeGCost, fCost); // Use constructor here
            }
        }
    }

    return {}; // Return an empty path if no path is found
}