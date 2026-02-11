#include "Environment.hpp"

namespace Planner
{
    Environment::Environment(Polygon perimeter) : perimeter(perimeter) {
                                                    // Add Validierungen, ist Perimeter geschlossen oder gibt es selbstüberscheidungen
                                                };

    void Environment::addObstacle(const Polygon &obs)
    {
        obstacles.push_back(obs);
    }

    void Environment::setVirtualWire(const LineString &wire)
    {
        virtualWire = wire;
    }

    void Environment::setDockingWire(const LineString &wire)
    {
        dockingWire = wire;
    }

    const Polygon& Environment::getPerimeter() const{
        return perimeter;
    }

    const std::vector<Polygon>& Environment::getObstacles() const{
        return obstacles;
    }
}