#include "Map.hpp"
#include "Robot.hpp"
#include <iostream>

int main() {
    // Initialisierung
    const std::size_t width = 20;
    const std::size_t height = 10;
    
    auto myMap = Map{width, height};
    auto myBot = Robot{0, 0};

    // Hindernisse hinzufügen
    for(std::size_t i = 0; i < 5; ++i) {
        myMap.setObstacle(10, i);
    }

    std::cout << "--- Coverage Planner Start ---" << std::endl;
    myBot.status();

    // Testbewegung
    myBot.move(1, 0, myMap);
    myBot.move(0, 1, myMap);
    
    myMap.print();
    myBot.status();

    return 0;
}