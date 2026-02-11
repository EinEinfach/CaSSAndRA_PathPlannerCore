#include "Geometry.hpp"
#include <algorithm>

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

    bool GeometryUtils::getIntersection(Point a, Point b, Point c, Point d, Point& out) {
        double s1_x = b.x - a.x;
        double s1_y = b.y - a.y;
        double s2_x = d.x - c.x;
        double s2_y = d.y - c.y;

        double s, t;
        double denom = (-s2_x * s1_y + s1_x * s2_y);

        // Wenn der Nenner 0 ist, dann sind die Linien parallel
        if (std::abs(denom) < 1e-10) return false;

        s = (-s1_y * (a.x - c.x) + s1_x * (a.y - c.y)) / denom;
        t = ( s2_x * (a.y - c.y) - s2_y * (a.x - c.x)) / denom;

        if (s >= 0 && s<= 1 && t >= 0 && t <= 1) {
            // Schnittpunkt gefunden
            out.x = a.x + (t * s1_x);
            out.y = a.y + (t * s1_y);
            return true;
        }
        return false;
    }

}