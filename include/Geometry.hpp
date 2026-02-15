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

        void rotate(double angleRad);

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
        static bool getIntersectionPoint(Point a, Point b, Point c, Point d, Point &out);

        // Prüft ob zwei Lineinsegmenten aufeinander liegen und wenn ja gibt es eine kolineare Linie zurück
        static bool getIntersectionLine(Point a, Point b, Point c, Point d, LineString &outline);

        // Prüft ob der Punkt innerhalb des Polygons liegt
        static bool isPointInPolygon(Point p, const Polygon &poly);

        // Rotiert einen Punkt um den Ursprung
        static Point rotatePoint(Point p, double angleRad);

        // Hilfsmethode zu Umrechnung Deg zu Rad
        static double degToRad(double deg);

        // Prüft ob die Linie einen Polygon schneidet
        static bool isLineIntersectingPolygon(Point p1, Point p2, const Polygon &poly);

        // Berechne Distanz zwischen zwei Punkten
        static double calculateDistance(Point a, Point b);
    };
};