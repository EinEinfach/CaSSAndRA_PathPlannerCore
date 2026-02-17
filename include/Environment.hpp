#pragma once
#include "Geometry.hpp"
#include <vector>

namespace Planner
{
    class Environment
    {
    public:
        Environment(Polygon perimeter); 
        void addObstacle(const Polygon &obs); 

        void setVirtualWire(const LineString &wire);
        void setDockingWire(const LineString &wire);

        const Polygon& getPerimeter() const;
        const std::vector<Polygon>& getObstacles() const;
        const LineString& getVirtualWire() const;

        void rotate(double angleRad);

    private:
        Polygon perimeter;
        std::vector<Polygon> obstacles;
        LineString virtualWire;
        LineString dockingWire;
    };
}