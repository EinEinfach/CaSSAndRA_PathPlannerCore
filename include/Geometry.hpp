#pragma once
#include <vector>
#include <initializer_list>

namespace Planner
{

    struct Point
    {
        double x;
        double y;

        bool operator==(const Point &other) const { return x == other.x && y == other.y; }
    };

    class LineString
    {
    public:
        LineString() = default;
        LineString(std::initializer_list<Point> pts);

        void addPoint(Point p);

        const std::vector<Point> &getPoints() const;

    protected:
        std::vector<Point> points;
    };

    class Polygon : public LineString
    {
    public:
        using LineString::LineString;
        bool isClosed() const;
    };

    class GeometryUtils
    {
    public:
        // Prüft, ob und wo sich zwei Liniensegmente (A-B und C-D) schneiden
        // Gibt true zurück, wenn sie sich schneiden, und schreibt das Ergebnis in intersectionPoint
        static bool getIntersection(Point a, Point b, Point c, Point d, Point &out);
    };
};