#pragma once
#include <vector>
#include <cstddef>

enum class CellState { Unvisited, Visited, Obstacle };

class Map {
public:
    Map(std::size_t w, std::size_t h);
    void setObstacle(std::size_t x, std::size_t y);
    void markVisited(std::size_t x, std::size_t y);
    void print() const;
    std::size_t getWidth() const { return width; }
    std::size_t getHeight() const { return height; }

private:
    std::size_t width, height;
    std::vector<std::vector<CellState>> grid;
};