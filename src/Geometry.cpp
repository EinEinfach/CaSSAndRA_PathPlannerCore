#include "Geometry.hpp"
#include <algorithm>
#include <cmath>

namespace Planner
{
    LineString::LineString(std::initializer_list<Point> pts) : points(pts)
    {
        // Hier können später Validierungen eingebaut werden
    }

    void LineString::addPoint(Point p)
    {
        points.push_back(p);
    }

    const std::vector<Point> &LineString::getPoints() const
    {
        return points;
    }

    void LineString::rotate(double angleRad) {
        for (auto& p : points) {
            p = GeometryUtils::rotatePoint(p, angleRad);
        }
    }

    bool Polygon::isClosed() const
    {
        return points.size() >= 3;
    }

    bool GeometryUtils::getIntersection(Point a, Point b, Point c, Point d, Point &out)
    {
        double s1_x = b.x - a.x;
        double s1_y = b.y - a.y;
        double s2_x = d.x - c.x;
        double s2_y = d.y - c.y;

        double s, t;
        double denom = (-s2_x * s1_y + s1_x * s2_y);

        // Wenn der Nenner 0 ist, dann sind die Linien parallel
        if (std::abs(denom) < 1e-10)
            return false;

        s = (-s1_y * (a.x - c.x) + s1_x * (a.y - c.y)) / denom;
        t = (s2_x * (a.y - c.y) - s2_y * (a.x - c.x)) / denom;

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Schnittpunkt gefunden
            out.x = a.x + (t * s1_x);
            out.y = a.y + (t * s1_y);
            return true;
        }
        return false;
    }

    bool GeometryUtils::isPointInPolygon(Point p, const Polygon &poly)
    {
        const auto &pts = poly.getPoints();
        bool inside = false;
        size_t numPoints = pts.size();

        for (size_t i = 0; i < numPoints; i++)
        {
            // j ist immer der Index vor i
            // Wenn i=0 ist, ist j der letzte Punkt (Modulo-Trick)
            size_t j = (i == 0) ? numPoints - 1 : i - 1;

            const Point &A = pts[i];
            const Point &B = pts[j];

            // Raycasting Check
            if (((A.y > p.y) != (B.y > p.y)) &&
                (p.x < (B.x - A.x) * (p.y - A.y) / (B.y - A.y) + A.x))
            {
                inside = !inside;
            }
        }
        return inside;
    }

    double GeometryUtils::degToRad(double deg) {
        return deg * M_PI / 180;
    }

    Point GeometryUtils::rotatePoint(Point p, double angleRad) {
        double cosA = std::cos(angleRad);
        double sinA = std::sin(angleRad);

        double newX = p.x * cosA - p.y * sinA;
        double newY = p.x * sinA + p.y * cosA;

        return {newX, newY};

    }

}