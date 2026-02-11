#include "Map.hpp" // Er findet das dank -Iinclude im Makefile
#include <iostream>

Map::Map(std::size_t w, std::size_t h) 
    : width(w), height(h), 
      grid(h, std::vector<CellState>(w, CellState::Unvisited)) {}

void Map::setObstacle(std::size_t x, std::size_t y) {
    if (x < width && y < height) grid[y][x] = CellState::Obstacle;
}

void Map::markVisited(std::size_t x, std::size_t y) {
    if (x < width && y < height && grid[y][x] != CellState::Obstacle) {
        grid[y][x] = CellState::Visited;
    }
}

void Map::print() const {
    for (const auto& row : grid) {
        for (const auto& cell : row) {
            switch (cell) {
                case CellState::Unvisited: std::cout << ". "; break;
                case CellState::Visited:   std::cout << "x "; break;
                case CellState::Obstacle:  std::cout << "# "; break;
            }
        }
        std::cout << "\n";
    }
}