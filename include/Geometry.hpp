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

        void setPoints(const std::vector<Point> &pts);

        const std::vector<Point> &getPoints() const;

        void rotate(double angleRad);

    protected:
        std::vector<Point> points;
    };

    class Polygon : public LineString
    {
    public:
        using LineString::LineString;
        Polygon(const std::vector<Point> &pts)
        {
            this->points = pts;
        }
        bool isClosed() const;
    };

    class GeometryUtils
    {
    public:
        // Prüft, ob und wo sich zwei Liniensegmente (A-B und C-D) schneiden
        // Gibt true zurück, wenn sie sich schneiden, und schreibt das Ergebnis in intersectionPoint
        static bool getIntersectionPoint(Point a, Point b, Point c, Point d, Point &out);

        // Prüft, ob und wo sich zwie Liniensegmente berühren
        static bool getTouchPoint(Point a, Point b, Point c, Point d, Point &out);

        // Prüft ob zwei Lineinsegmenten aufeinander liegen und wenn ja gibt es eine kolineare Linie zurück
        static bool getIntersectionLine(Point a, Point b, Point c, Point d, LineString &outline);

        // Prüft ob der Punkt innerhalb des Polygons liegt (inkl Kanten)
        static bool isPointCoveredByPolygon(Point p, const Polygon &poly);

        // Prüft ob der Punkt innerhalb des Polygons liegt (ohne Kanten)
        static bool isPointInsidePolygon(Point p, const Polygon &poly);

        // Rotiert einen Punkt um den Ursprung
        static Point rotatePoint(Point p, double angleRad);

        // Hilfsmethode zu Umrechnung Deg zu Rad
        static double degToRad(double deg);

        // Prüft ob die Linie einen Polygon schneidet
        static bool isLineIntersectingPolygon(Point p1, Point p2, const Polygon &poly);

        // Prüft ob die Linie komplet im Polygon (inkl Kanten)
        static bool isLineCoveredByPolygon(Point p1, Point p2, const Polygon &poly);

        // Prüft ob die Linie komplet im Polygon (ohne Kante)
        static bool isLineInsidePolygon(Point p1, Point p2, const Polygon &poly);

        // Prüft ob ein Polygon schneidet ein anderes
        static bool isPolygonIntersectingPolygon(const Polygon &poly1, const Polygon &poly2);

        // Prüft ob ein Polygon komplet in einem anderen Polygon liegt
        static bool isPolygonCoveredByPolygon(const Polygon &poly1, const Polygon &poly2);

        // Berechne Distanz zwischen zwei Punkten
        static double calculateDistance(Point a, Point b);

        // Berechne Fläche
        static double calculateSignedArea(const LineString &poly);

        // Berechne die Orientierung (CW, CCW)
        static void ensureOrientation(LineString &poly, bool ccw);
    };
};