#include "Environment.hpp"

namespace Planner
{
    Environment::Environment(Polygon perimeter) : perimeter(perimeter) {
        // Perimeter MUSS CCW sein
        GeometryUtils::ensureOrientation(this->perimeter, true);
    }

    void Environment::addObstacle(const Polygon &obs)
    {
        Polygon orientedObs = obs;
        // Obstacles MÜSSEN CW (Uhrzeigersinn) sein
        GeometryUtils::ensureOrientation(orientedObs, false);
        obstacles.push_back(orientedObs);
    }

    void Environment::addMowArea(const Polygon &mowArea)
    {
        Polygon orientedArea = mowArea;
        // MowAreas sind Teilbereiche des Perimeters -> CCW
        GeometryUtils::ensureOrientation(orientedArea, true);
        mowAreas.push_back(orientedArea);
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

    const std::vector<Polygon>& Environment::getMowAreas() const{
        return mowAreas;
    }

    const LineString& Environment::getVirtualWire() const{
        return virtualWire;
    }

    const LineString& Environment::getDockingWire() const{
        return dockingWire;
    }

    void Environment::rotate(double angleRad) {
        perimeter.rotate(angleRad);
        
        for (auto& obs : obstacles) {
            obs.rotate(angleRad);
        }

        for (auto& mowArea : mowAreas) {
            mowArea.rotate(angleRad);
        }

        virtualWire.rotate(angleRad);
        dockingWire.rotate(angleRad);
    }
}