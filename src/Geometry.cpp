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

    void LineString::rotate(double angleRad)
    {
        for (auto &p : points)
        {
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

    double GeometryUtils::degToRad(double deg)
    {
        return deg * M_PI / 180;
    }

    Point GeometryUtils::rotatePoint(Point p, double angleRad)
    {
        double cosA = std::cos(angleRad);
        double sinA = std::sin(angleRad);

        double newX = p.x * cosA - p.y * sinA;
        double newY = p.x * sinA + p.y * cosA;

        return {newX, newY};
    }

    bool GeometryUtils::isLineIntersectingPolygon(Point p1, Point p2, const Polygon &poly)
    {
        const auto &pts = poly.getPoints();
        size_t numPoints = pts.size();
        if (numPoints < 3)
            return false;

        // --- TEIL 1: ECHTE SCHNITTPUNKTE PRÜFEN ---
        for (size_t i = 0; i < numPoints; ++i)
        {
            Point intersect;
            // Prüfe das Liniensegment p1-p2 gegen die Polygon-Kante (pts[i] bis pts[next])
            if (getIntersection(p1, p2, pts[i], pts[(i + 1) % numPoints], intersect))
            {
                // Ein Schnittpunkt blockiert nur, wenn er NICHT fast identisch
                // mit den Endpunkten p1 oder p2 ist (Endpunkt-Toleranz 1mm)
                double distToStart = std::sqrt(std::pow(intersect.x - p1.x, 2) + std::pow(intersect.y - p1.y, 2));
                double distToEnd = std::sqrt(std::pow(intersect.x - p2.x, 2) + std::pow(intersect.y - p2.y, 2));

                if (distToStart > 0.001 && distToEnd > 0.001)
                {
                    return true; // Echter Durchschuss durch eine Kante
                }
            }
        }

        // --- TEIL 2: VOLLSTÄNDIGE LAGE IM HINDERNIS PRÜFEN ---
        // Wir prüfen den Mittelpunkt, um Linien zu finden, die komplett "drinnen" liegen.
        Point midPoint = {(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0};

        if (isPointInPolygon(midPoint, poly))
        {
            // Falls isPointInPolygon "true" sagt, prüfen wir, ob wir nur auf der Kante liegen.
            // Wir berechnen den minimalen Abstand des midPoints zu allen Kanten des Polygons.
            double minDistanceToEdge = 1e18; // Startwert unendlich

            for (size_t i = 0; i < numPoints; ++i)
            {
                Point A = pts[i];
                Point B = pts[(i + 1) % numPoints];

                // Abstand Punkt zu Segment (A-B)
                double l2 = std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2);
                double t = ((midPoint.x - A.x) * (B.x - A.x) + (midPoint.y - A.y) * (B.y - A.y)) / l2;
                t = std::max(0.0, std::min(1.0, t));

                Point projection = {A.x + t * (B.x - A.x), A.y + t * (B.y - A.y)};
                double dist = std::sqrt(std::pow(midPoint.x - projection.x, 2) + std::pow(midPoint.y - projection.y, 2));

                if (dist < minDistanceToEdge)
                    minDistanceToEdge = dist;
            }

            // Wenn der midPoint näher als 1mm an einer Kante liegt, werten wir das
            // als "auf der Kante gleitend" und NICHT als blockiert.
            if (minDistanceToEdge < 0.001)
            {
                return false;
            }

            return true; // Der Punkt ist wirklich signifikant tief im Hindernis
        }

        return false;
        // const auto &pts = poly.getPoints();
        // if (pts.size() < 3)
        //     return false;
        // // 1. Check auf echte Schnittpunkte
        // for (size_t i = 0; i < pts.size(); ++i)
        // {
        //     Point intersect;
        //     // Prüfe Segment p1-p2 gegen jede Kante des Hindernisses
        //     if (getIntersection(p1, p2, pts[i], pts[(i + 1) % pts.size()], intersect))
        //     {
        //         // Ein Schnittpunkt wurde gefunden -> Weg ist blockiert!
        //         return true;
        //     }
        // }
        // // 2. Check liegt die gesamte Linie im Hindernis
        // Point midPoint = {(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0};
        // if (isPointInPolygon(midPoint, poly)) {
        //     return true;
        // }
        // return false;
    }

    double GeometryUtils::calculateDistance(Point a, Point b)
    {
        return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
    }
}