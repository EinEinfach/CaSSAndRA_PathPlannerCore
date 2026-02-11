#pragma once
#include "Map.hpp"

class Robot {
public:
    Robot(std::size_t startX, std::size_t startY);
    void move(int dx, int dy, Map& map);
    void status() const;

private:
    std::size_t x, y;
};