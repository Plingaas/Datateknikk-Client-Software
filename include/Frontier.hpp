#ifndef FRONTIERMAP_HPP
#define FRONTIERMAP_HPP

#include <vector>
#include <queue>
#include <string>
#include <iostream>
#include <memory>
#include "threepp/threepp.hpp"
#include <simple_socket/SimpleConnection.hpp>
#include <utility>
#include <set>
#include <map>
#include "Datatypes.hpp"

using Position = std::pair<int, int>;

struct Frontier {
    std::vector<Cell> cells;
    Cell center;
};

class GridMap {

public:
    GridMap(int size, double resolution)

        : size_(size), resolution_(resolution), origin_(size / 2, size / 2),
          robot_position_(size / 2, size / 2), grid_(size, std::string(size, '-')) {

    }

    void set_grid(const std::vector<std::string> &grid) {
        this->grid_ = grid;
    }

    char get_point_on_grid(int i, int j) {
        auto val = this ->grid_[i][j];
        return val;

    }
    std::pair<double, double> get_roboPos() {
        auto pos = std::make_pair(x_, y_);
        return pos;
    }

    // Perform BFS to find frontier areas
    std::set<Position> findFrontier() {
        std::set<Position> frontierAreas;
        std::queue<Position> q;
        std::set<Position> visited;

        Position start = {static_cast<int>(x_), static_cast<int>(y_)};
        q.push(start);
        visited.insert(start);

        // Directions for neighbors (up, down, left, right)
        const std::vector<Position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1},{-1,-1},{-1,1},{1,-1},{1,1}};

        while (!q.empty()) {
            auto [currentRow, currentCol] = q.front();
            q.pop();

            // If the current cell is a frontier, add it to the result
            if (isFrontier(currentRow, currentCol)) {
                frontierAreas.insert({currentRow, currentCol});
            }

            // Explore neighbors
            for (const auto &[dr, dc] : directions) {
                int newRow = currentRow + dr;
                int newCol = currentCol + dc;

                if (isValidPosition(newRow, newCol) && visited.find({newRow, newCol}) == visited.end()) {
                    visited.insert({newRow, newCol});
                    if (grid_[newRow][newCol] == '0') {
                        q.push({newRow, newCol});
                    }
                }
            }
        }

        return frontierAreas;
    }

        // Find the middle points of the three largest frontiers
    std::vector<Position> findLargestFrontierCenters() {
        std::vector<std::vector<Position>> frontiers;
        std::set<Position> visited;

        // Directions for neighbors (up, down, left, right)
        const std::vector<Position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1},{-1,-1},{-1,1},{1,-1},{1,1}};

        for (int row = 0; row < grid_.size(); ++row) {
            for (int col = 0; col < grid_[0].size(); ++col) {
                Position start = {row, col};
                if (isFrontier(row, col) && visited.find(start) == visited.end()) {
                    // Perform BFS to collect all connected frontier points
                    std::vector<Position> frontier;
                    std::queue<Position> q;
                    q.push(start);
                    visited.insert(start);

                    while (!q.empty()) {
                        auto [currentRow, currentCol] = q.front();
                        q.pop();
                        frontier.push_back({currentRow, currentCol});

                        for (const auto &[dr, dc] : directions) {
                            int newRow = currentRow + dr;
                            int newCol = currentCol + dc;

                            Position neighbor = {newRow, newCol};
                            if (isValidPosition(newRow, newCol) && isFrontier(newRow, newCol) && visited.find(neighbor) == visited.end()) {
                                visited.insert(neighbor);
                                q.push(neighbor);
                            }
                        }
                    }

                    // Add the collected frontier to the list of frontiers
                    frontiers.push_back(frontier);
                }
            }
        }

        // Sort the frontiers by size in descending order
        std::sort(frontiers.begin(), frontiers.end(), [](const auto &a, const auto &b) {
            return a.size() > b.size();
        });

        // Find the middle points of the three largest frontiers
        std::vector<Position> centers;
        for (int i = 0; i < std::min(1, static_cast<int>(frontiers.size())); ++i) {
            const auto &frontier = frontiers[i];
            centers.push_back(frontier[frontier.size() / 2]); // Middle point
        }

        return centers;
    }





    [[nodiscard]] std::vector<std::string> grid() const {
        return grid_;
    }





private:
    int size_;
    double resolution_;
    std::pair<double, double> robot_position_;
    std::pair<double, double> origin_;
    std::vector<std::string> grid_;

    std::vector<std::pair<double, double>> scanframe_;
    double x_;
    double y_;
    double heading_=0;

    // Check if a position is within the bounds of the grid


    // Check if a 0 borders a '-'
};





#endif //FRONTIERMAP_HPP
