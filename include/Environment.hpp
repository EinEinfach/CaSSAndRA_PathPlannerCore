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
        void addMowArea(const Polygon &mowArea);

        void setVirtualWire(const LineString &wire);
        void setDockingWire(const LineString &wire);

        const Polygon& getPerimeter() const;
        const std::vector<Polygon>& getObstacles() const;
        const std::vector<Polygon>& getMowAreas() const;
        const LineString& getVirtualWire() const;
        const LineString& getDockingWire() const;

        void rotate(double angleRad);

    private:
        Polygon perimeter;
        std::vector<Polygon> obstacles;
        std::vector<Polygon> mowAreas;
        LineString virtualWire;
        LineString dockingWire;
    };
}