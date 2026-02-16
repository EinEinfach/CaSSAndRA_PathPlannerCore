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

    bool GeometryUtils::getIntersectionPoint(Point a, Point b, Point c, Point d, Point &out)
    {
        double s1_x = b.x - a.x;
        double s1_y = b.y - a.y;
        double s2_x = d.x - c.x;
        double s2_y = d.y - c.y;

        double denom = (-s2_x * s1_y + s1_x * s2_y);

        if (std::abs(denom) < 1e-10)
            return false;

        // s und t berechnen
        double s = (-s1_y * (a.x - c.x) + s1_x * (a.y - c.y)) / denom;
        double t = (s2_x * (a.y - c.y) - s2_y * (a.x - c.x)) / denom;

        // EPSILON für "Nicht-Berühren"
        // Wir nutzen einen winzigen Schwellenwert, um die Endpunkte zu ignorieren.
        double eps = 1e-9;

        if (s > eps && s < (1.0 - eps) && t > eps && t < (1.0 - eps))
        {
            out.x = a.x + (t * s1_x);
            out.y = a.y + (t * s1_y);
            return true;
        }

        return false;
    }

    bool GeometryUtils::getIntersectionLine(Point a, Point b, Point c, Point d, LineString &outLine)
    {
        // 1. Richtungsvektoren
        double dx1 = b.x - a.x;
        double dy1 = b.y - a.y;
        double dx2 = d.x - c.x;
        double dy2 = d.y - c.y;

        // 2. STRENGER CHECK: Parallelität
        // Wenn das Kreuzprodukt nicht 0 ist, schneiden sie sich (crossing)
        // oder sind windschief, aber sie liegen nicht aufeinander.
        double denom = dx1 * dy2 - dy1 * dx2;
        if (std::abs(denom) > 1e-9)
        {
            return false; // Kreuzende Linien -> laut Anforderung false
        }

        // 3. CHECK: Liegen sie auf der exakt gleichen Geraden?
        // Kreuzprodukt zwischen Vektor (a->b) und (a->c) muss 0 sein
        double distCheck = dx1 * (c.y - a.y) - dy1 * (c.x - a.x);
        if (std::abs(distCheck) > 1e-9)
        {
            return false; // Parallel, aber versetzt (kein Kontakt)
        }

        // 4. Projektion auf die Achse der ersten Linie (Skalarprodukt)
        auto project = [](Point p, Point start, double dx, double dy)
        {
            return (p.x - start.x) * dx + (p.y - start.y) * dy;
        };

        double tA = project(a, a, dx1, dy1);
        double tB = project(b, a, dx1, dy1);
        double tC = project(c, a, dx1, dy1);
        double tD = project(d, a, dx1, dy1);

        double start1 = std::min(tA, tB);
        double end1 = std::max(tA, tB);
        double start2 = std::min(tC, tD);
        double end2 = std::max(tC, tD);

        // Schnittmenge der 1D-Intervalle
        double overlapStart = std::max(start1, start2);
        double overlapEnd = std::min(end1, end2);

        // 5. Nur wenn die Überlappung eine echte Länge hat
        // (eps verhindert, dass eine bloße Punkt-Berührung als Linie zählt)
        double eps = 1e-7;
        if (overlapStart < overlapEnd - eps)
        {
            // Wir sammeln alle Punkte, die im Überlappungsbereich liegen
            std::vector<Point> allPoints = {a, b, c, d};
            std::vector<Point> resultPoints;

            for (const auto &p : allPoints)
            {
                double t = project(p, a, dx1, dy1);
                if (t >= overlapStart - eps && t <= overlapEnd + eps)
                {
                    // Duplikate vermeiden
                    bool exists = false;
                    for (const auto &rp : resultPoints)
                    {
                        if (std::abs(rp.x - p.x) < eps && std::abs(rp.y - p.y) < eps)
                            exists = true;
                    }
                    if (!exists)
                        resultPoints.push_back(p);
                }
            }

            if (resultPoints.size() >= 2)
            {
                // Sortieren für konsistenten LineString (z.B. von links nach rechts)
                std::sort(resultPoints.begin(), resultPoints.end(), [](Point p1, Point p2)
                          { return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y); });

                outLine.addPoint(resultPoints.front());
                outLine.addPoint(resultPoints.back());
                return true;
            }
        }

        return false;
    }

    bool GeometryUtils::isPointInPolygon(Point p, const Polygon &poly)
    {
        const auto &pts = poly.getPoints();
        size_t numPoints = pts.size();
        if (numPoints < 3)
            return false;

        bool inside = false;
        for (size_t i = 0; i < numPoints; i++)
        {
            size_t j = (i == 0) ? numPoints - 1 : i - 1;
            const Point &A = pts[i];
            const Point &B = pts[j];

            // --- CHECK: Liegt der Punkt exakt auf der Kante oder einem Eckpunkt? ---
            // Wir prüfen, ob der Punkt p auf dem Segment AB liegt.
            // 1. Er muss kollinear sein (Fläche des Dreiecks ABP fast 0)
            // 2. Er muss innerhalb der Bounds von A und B liegen
            double crossProduct = (p.y - A.y) * (B.x - A.x) - (p.x - A.x) * (B.y - A.y);
            if (std::abs(crossProduct) < 1e-9)
            { // Kollinear
                if (p.x >= std::min(A.x, B.x) - 1e-9 && p.x <= std::max(A.x, B.x) + 1e-9 &&
                    p.y >= std::min(A.y, B.y) - 1e-9 && p.y <= std::max(A.y, B.y) + 1e-9)
                {
                    return true; // Punkt liegt auf Kante oder Ecke
                }
            }

            // --- Standard Raycasting (für Punkte, die nicht auf der Kante liegen) ---
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

        // --- ECHTE SCHNITTPUNKTE PRÜFEN ---
        for (size_t i = 0; i < numPoints; ++i)
        {
            Point intersect;
            // Prüfe das Liniensegment p1-p2 gegen die Polygon-Kante (pts[i] bis pts[next])
            if (getIntersectionPoint(p1, p2, pts[i], pts[(i + 1) % numPoints], intersect))
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
        return false;
    }

    bool GeometryUtils::isLineCoverdByPolygon(Point p1, Point p2, const Polygon &poly)
    {
        const auto &pts = poly.getPoints();
        size_t numPoints = pts.size();
        if (numPoints < 3)
            return false;

        // 1. Endpunkte prüfen
        // Da isPointInPolygon bei uns "true" für Kanten/Ecken liefert,
        // ist Bedingung 3 (Berühren erlaubt) hier bereits abgedeckt.
        if (!isPointInPolygon(p1, poly) || !isPointInPolygon(p2, poly))
        {
            return false;
        }

        // 2. Mittelpunkt-Check (Grobauswahl)
        Point midPoint = {(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0};
        if (!isPointInPolygon(midPoint, poly))
        {
            return false; // Liegt bei konkaven Polygonen "im Freien"
        }

        // 3. Echte Schnitte nach außen verhindern
        // Ein "Durchschuss" durch eine Kante würde bedeuten, die Linie verlässt das Polygon.
        for (size_t i = 0; i < numPoints; ++i)
        {
            Point intersect;
            if (getIntersectionPoint(p1, p2, pts[i], pts[(i + 1) % numPoints], intersect))
            {
                return false;
            }
        }

        // 4. Spezialfall: Liegt die Linie exakt auf einer Kante? (Bedingung 2)
        // Wir prüfen, ob unsere Linie p1-p2 eine der Polygonkanten überlappt.
        bool isOnEdge = false;
        for (size_t i = 0; i < numPoints; ++i)
        {
            LineString overlap;
            if (getIntersectionLine(p1, p2, pts[i], pts[(i + 1) % numPoints], overlap))
            {
                // Wenn die Überlappung exakt so lang ist wie unsere Linie p1-p2,
                // dann liegt sie vollständig auf der Kante.
                double dOrig = calculateDistance(p1, p2);
                const auto &oPts = overlap.getPoints();
                double dOverlap = calculateDistance(oPts.front(), oPts.back());

                if (std::abs(dOrig - dOverlap) < 1e-7)
                {
                    isOnEdge = true;
                    break;
                }
            }
        }

        // Ergebnis:
        // Wenn wir hier ankommen, wissen wir: Endpunkte sind drin, kein Schnitt nach außen.
        // Das bedeutet: Die Linie ist entweder komplett IM Polygon oder auf der Kante.
        // Beides soll laut deiner Anforderung TRUE liefern.
        return true;
    }

    double GeometryUtils::calculateDistance(Point a, Point b)
    {
        return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
    }
}