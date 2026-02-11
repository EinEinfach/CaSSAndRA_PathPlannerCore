#include "Enviroment.hpp"

namespace Planner
{
    Enviroment::Enviroment(Polygon perimeter) : perimeter(perimeter) {
                                                    // Add Validierungen, ist Perimeter geschlossen oder gibt es selbstüberscheidungen
                                                };

    void Enviroment::addObstacle(const Polygon &obs)
    {
        obstacles.push_back(obs);
    }

    void Enviroment::setVirtualWire(const LineString &wire)
    {
        virtualWire = wire;
    }

    void Enviroment::setDockingWire(const LineString &wire)
    {
        dockingWire = wire;
    }

    const Polygon& Enviroment::getPerimeter() const{
        return perimeter;
    }

    const std::vector<Polygon>& Enviroment::getObstacles() const{
        return obstacles;
    }
}