#include "Robot.hpp"
#include <iostream>

Robot::Robot(std::size_t startX, std::size_t startY) : x(startX), y(startY) {}

void Robot::move(int dx, int dy, Map& map) {
    // Einfache Bounds-Checks (Modernes C++: static_cast für Typumwandlung)
    int nextX = static_cast<int>(x) + dx;
    int nextY = static_cast<int>(y) + dy;

    if (nextX >= 0 && nextX < static_cast<int>(map.getWidth()) &&
        nextY >= 0 && nextY < static_cast<int>(map.getHeight())) {
        x = static_cast<std::size_t>(nextX);
        y = static_cast<std::size_t>(nextY);
        map.markVisited(x, y);
    }
}

void Robot::status() const {
    std::cout << "Robot Position: [" << x << ", " << y << "]\n";
}