#include "Geometry.hpp"

namespace Planner {
    LineString::LineString(std::initializer_list<Point> pts) : points(pts) {
        // Hier können später Validierungen eingebaut werden
    }

    void LineString::addPoint(Point p) {
        points.push_back(p);
    }

    const std::vector<Point>& LineString::getPoints() const {
        return points;
    }

    bool Polygon::isClosed() const {
        return points.size() >= 3;
    }

}