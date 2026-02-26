#include <gtest/gtest.h>
#include "Geometry.hpp"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using namespace Planner;

// ******************GeometryUtils::getIntersectionPoint****************

// 1. Test: Ein klassischer Kreuz-Schnitt
TEST(GeometryUtilsTest, GetIntersection_Cross)
{
    Point a{0, 0}, b{10, 10};
    Point c{0, 10}, d{10, 0};
    Point out;

    bool result = GeometryUtils::getIntersectionPoint(a, b, c, d, out);

    EXPECT_TRUE(result);
    EXPECT_NEAR(out.x, 5.0, 1e-7);
    EXPECT_NEAR(out.y, 5.0, 1e-7);
}

// 2. Test: Parallele Linien (sollten keinen Schnittpunkt haben)
TEST(GeometryUtilsTest, GetIntersection_Parallel)
{
    Point a{0, 0}, b{10, 0};
    Point c{0, 1}, d{10, 1};
    Point out;

    bool result = GeometryUtils::getIntersectionPoint(a, b, c, d, out);

    EXPECT_FALSE(result);
}

// 3. Test: Berührung am Endpunkt (Soll false liefern laut deiner Definition)
TEST(GeometryUtilsTest, GetIntersection_EndpointTouch)
{
    Point a{0, 0}, b{5, 5};
    Point c{5, 5}, d{10, 5};
    Point out;

    // Wir erwarten FALSE, weil Berührung kein "Durchschuss" ist
    bool result = GeometryUtils::getIntersectionPoint(a, b, c, d, out);
    EXPECT_FALSE(result);
}

// 4. Test: Linien liegen aufeinander (Kollinear - Überlappend)
TEST(GeometryUtilsTest, GetIntersection_CollinearOverlapping)
{
    Point a{0, 0}, b{10, 0};
    Point c{5, 0}, d{15, 0};
    Point out;

    // Kollineare Linien haben unendlich viele Schnittpunkte oder liegen aufeinander.
    // Mathematisch ist der Nenner (denom) hier 0.
    // Da kein eindeutiger "Durchschuss" existiert, sollte dies FALSE liefern.
    EXPECT_FALSE(GeometryUtils::getIntersectionPoint(a, b, c, d, out));
}

// 5. Test: Linien auf der gleichen Geraden, aber getrennt (Kollinear - Getrennt)
TEST(GeometryUtilsTest, GetIntersection_CollinearSeparate)
{
    Point a{0, 0}, b{5, 0};
    Point c{10, 0}, d{15, 0};
    Point out;

    EXPECT_FALSE(GeometryUtils::getIntersectionPoint(a, b, c, d, out));
}

// ******************GeometryUtils::getTouchPoint**********************

// 1. T-Kreuzung (Berührung am Endpunkt von CD auf der Strecke AB)
TEST(GeometryUtilsTest, GetTouchPoint_LineTouch)
{
    Point out;
    EXPECT_TRUE(GeometryUtils::getTouchPoint({0, 0}, {10, 0}, {5, 0}, {5, 5}, out));
}

// 2. Linie kreuzt die andere
TEST(GeometryUtilsTest, GetTouchPoint_LineCross)
{
    Point out;
    EXPECT_FALSE(GeometryUtils::getTouchPoint({0, 0}, {10, 10}, {0, 10}, {10, 0}, out));
}

// 3. Linien berühren sich am Ende
TEST(GeometryUtilsTest, GetTouchPoint_LineTouchAtEnd)
{
    Point out;
    EXPECT_TRUE(GeometryUtils::getTouchPoint({0, 0}, {10, 0}, {10, 0}, {10, 10}, out));
}

// ******************GeometryUtils::getIntersectionLine****************

// Test 1: Überlapung zum Teil
TEST(GeometryUtilsTest, GetIntersectionLine_Overlap)
{
    Point a{0, 0}, b{10, 0};
    Point c{5, 0}, d{15, 0}; // Überlappt von 5 bis 10
    LineString out;

    bool result = GeometryUtils::getIntersectionLine(a, b, c, d, out);

    ASSERT_TRUE(result);
    auto pts = out.getPoints();
    ASSERT_EQ(pts.size(), 2UL);

    // Wir erwarten das Segment von (5,0) bis (10,0)
    EXPECT_NEAR(pts[0].x, 5.0, 1e-7);
    EXPECT_NEAR(pts[1].x, 10.0, 1e-7);
    EXPECT_NEAR(pts[0].y, 0.0, 1e-7);
    EXPECT_NEAR(pts[1].y, 0.0, 1e-7);
}

// Test 2: Überlapung gleich
TEST(GeometryUtilsTest, GetIntersectionLine_OverlapEqual)
{
    Point a{-5, -5}, b{10, 10};
    Point c{-5, -5}, d{10, 10}; // Überlappt gleich
    LineString out;

    bool result = GeometryUtils::getIntersectionLine(a, b, c, d, out);

    ASSERT_TRUE(result);
    auto pts = out.getPoints();
    ASSERT_EQ(pts.size(), 2UL);

    // Wir erwarten das Segment von (5,0) bis (10,0)
    EXPECT_NEAR(pts[0].x, -5.0, 1e-7);
    EXPECT_NEAR(pts[1].x, 10.0, 1e-7);
    EXPECT_NEAR(pts[0].y, -5.0, 1e-7);
    EXPECT_NEAR(pts[1].x, 10.0, 1e-7);
}

// Test 3: Keine Überlapung
TEST(GeometryUtilsTest, GetIntersectionLine_NoOverlap)
{
    Point a{-5, -5}, b{10, 10};
    Point c{-6, -7}, d{10, 10};
    LineString out;

    bool result = GeometryUtils::getIntersectionLine(a, b, c, d, out);

    EXPECT_FALSE(result);
}

// ******************GeometryUtils::isPointCoveredByPolygon*******************

// Hilfs-Polygon erstellen (Quadrat von 0,0 bis 10,10)
Polygon createSquare()
{
    Polygon poly;
    poly.addPoint({-10, -10});
    poly.addPoint({-10, 10});
    poly.addPoint({10, 10});
    poly.addPoint({10, -10});
    return poly;
}

// 1. Punkt klar innerhalb
TEST(GeometryUtilsTest, IsCoveredByPolygon_Inside)
{
    Polygon poly = createSquare();
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({5, 5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({-5, -5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({5, -5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({-5, 5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({0, 0}, poly));
}

// 2. Punkt klar außerhalb
TEST(GeometryUtilsTest, IsPointCoveredByPolygon_Outside)
{
    Polygon poly = createSquare();
    EXPECT_FALSE(GeometryUtils::isPointCoveredByPolygon({15, 5}, poly));
    EXPECT_FALSE(GeometryUtils::isPointCoveredByPolygon({-15, 5}, poly));
    EXPECT_FALSE(GeometryUtils::isPointCoveredByPolygon({5, 15}, poly));
    EXPECT_FALSE(GeometryUtils::isPointCoveredByPolygon({5, -15}, poly));
}

// 3. Punkt exakt auf der Kante (Anforderung: true)
TEST(GeometryUtilsTest, IsPointCoveredByPolygon_OnEdge)
{
    Polygon poly = createSquare();
    // Punkt auf der unteren Kante
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({10, 0}, poly));
}

// 4. Punkt exakt auf einem Eckpunkt (Anforderung: true)
TEST(GeometryUtilsTest, IsPointCoveredPolygon_OnVertex)
{
    Polygon poly = createSquare();
    // Direkt auf der Ecke (10, 10)
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({10, 10}, poly));
}

// 5. Test mit konkaven Polygon
TEST(GeometryUtilsTest, IsPointCoveredByPolygon_Concave)
{
    // L-Form
    Polygon lShape;
    lShape.addPoint({0, 0});
    lShape.addPoint({10, 0});
    lShape.addPoint({10, 5});
    lShape.addPoint({5, 5});
    lShape.addPoint({5, 10});
    lShape.addPoint({0, 10});

    // Dieser Punkt (7, 7) liegt im "leeren Eck" des Ls, also AUSSERHALB
    EXPECT_FALSE(GeometryUtils::isPointCoveredByPolygon({7, 7}, lShape));
    // Dieser Punkt (2, 2) liegt INNERHALB
    EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon({2, 2}, lShape));
}

// ******************GeometryUtils::isPointInsidePolygon*******************

// 1. Punkt klar innerhalb
TEST(GeometryUtilsTest, IsPointInsidePolygon_Inside)
{
    Polygon poly = createSquare();
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({5, 5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({-5, -5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({5, -5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({-5, 5}, poly));
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({0, 0}, poly));
}

// 2. Punkt klar außerhalb
TEST(GeometryUtilsTest, IsPointInsidePolygon_Outside)
{
    Polygon poly = createSquare();
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({15, 5}, poly));
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({-15, 5}, poly));
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({5, 15}, poly));
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({5, -15}, poly));
}

// 3. Punkt exakt auf der Kante (Anforderung: true)
TEST(GeometryUtilsTest, IsPointInsidePolygon_OnEdge)
{
    Polygon poly = createSquare();
    // Punkt auf der unteren Kante
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({10, 0}, poly));
}

// 4. Punkt exakt auf einem Eckpunkt (Anforderung: true)
TEST(GeometryUtilsTest, IsPointInsidePolygon_OnVertex)
{
    Polygon poly = createSquare();
    // Direkt auf der Ecke (10, 10)
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({10, 10}, poly));
}

// 5. Test mit konkaven Polygon
TEST(GeometryUtilsTest, IsPointInsidePolygon_Concave)
{
    // L-Form
    Polygon lShape;
    lShape.addPoint({0, 0});
    lShape.addPoint({10, 0});
    lShape.addPoint({10, 5});
    lShape.addPoint({5, 5});
    lShape.addPoint({5, 10});
    lShape.addPoint({0, 10});

    // Dieser Punkt (7, 7) liegt im "leeren Eck" des Ls, also AUSSERHALB
    EXPECT_FALSE(GeometryUtils::isPointInsidePolygon({7, 7}, lShape));
    // Dieser Punkt (2, 2) liegt INNERHALB
    EXPECT_TRUE(GeometryUtils::isPointInsidePolygon({2, 2}, lShape));
}

// ******************GeometryUtils::isLineIntersectingPolygon***************

// 1. Test: kompletter durchschuss
TEST(GeometryUtilsTest, IsLineIntersectingPolygon_ThroughPass)
{
    Polygon obstacle = createSquare();

    // Linie geht von links (-15, 5) nach rechts (15, 5) mitten durch
    Point p1{-15, 5}, p2{15, 5};
    EXPECT_TRUE(GeometryUtils::isLineIntersectingPolygon(p1, p2, obstacle));

    // Linie liegt komplett AUSSERHALB
    Point p3{-5, -20}, p4{15, -25};
    EXPECT_FALSE(GeometryUtils::isLineIntersectingPolygon(p3, p4, obstacle));
}

// 2. Test: Glitching
TEST(GeometryUtilsTest, IsLineIntersectingPolygon_EdgeGliding)
{
    Polygon obstacle = createSquare(); // 0,0 bis 10,10

    // Linie liegt exakt auf der Oberkante (0,10) bis (10,10)
    Point p1{0, 10}, p2{10, 10};
    // Erwartung: FALSE, da wir nur "berühren/gleiten" und nicht "durchschießen"
    EXPECT_FALSE(GeometryUtils::isLineIntersectingPolygon(p1, p2, obstacle));
}

// ******************GeometryUtils::isLineCoverdByPolygon***************

// Test 1: Line im Polygon
TEST(GeometryUtilsTest, IsLineCoveredByPolygon_LineInside)
{
    Polygon polygon = createSquare();
    Point a = {-5.0, -5.0};
    Point b = {5.0, 5.0};

    EXPECT_TRUE(GeometryUtils::isLineCoveredByPolygon(a, b, polygon));
}

// Test 2: Line im Polygon, aber berührt die Kante
TEST(GeometryUtilsTest, IsLineCoveredByPolygon_LineInsideTouchEdge)
{
    Polygon polygon = createSquare();
    Point a = {-5.0, -10.0};
    Point b = {5.0, 5.0};

    EXPECT_TRUE(GeometryUtils::isLineCoveredByPolygon(a, b, polygon));
}

// Test 3: Line im Polygon, aber berührt die Polygonkoordinate
TEST(GeometryUtilsTest, IsLineCoveredByPolygon_LineInsideTouchCoord)
{
    Polygon polygon = createSquare();
    Point a = {-10.0, -10.0};
    Point b = {5.0, 5.0};

    EXPECT_TRUE(GeometryUtils::isLineCoveredByPolygon(a, b, polygon));
}

// Test 4: Line außerhalb vom Polygon
TEST(GeometryUtilsTest, IsLineCoveredByPolygon_LineOutside)
{
    Polygon polygon = createSquare();
    Point a = {-50.0, -50.0};
    Point b = {50.0, -50.0};

    EXPECT_FALSE(GeometryUtils::isLineCoveredByPolygon(a, b, polygon));
}

// Test 5: Line kruezt Polygon
TEST(GeometryUtilsTest, IsLineCoveredByPolygon_LineCross)
{
    Polygon polygon = createSquare();
    Point a = {-50.0, -50.0};
    Point b = {50.0, 50.0};

    EXPECT_FALSE(GeometryUtils::isLineCoveredByPolygon(a, b, polygon));
}

// ******************GeometryUtils::isLineInsidePolygon***************

// Test 1: Line im Polygon
TEST(GeometryUtilsTest, IsLineInsidePolygon_LineInside)
{
    Polygon polygon = createSquare();
    Point a = {-5.0, -5.0};
    Point b = {5.0, 5.0};

    EXPECT_TRUE(GeometryUtils::isLineInsidePolygon(a, b, polygon));
}

// Test 2: Line im Polygon, aber berührt die Kante
TEST(GeometryUtilsTest, IsLineInsidePolygon_LineInsideTouchEdge)
{
    Polygon polygon = createSquare();
    Point a = {-5.0, -10.0};
    Point b = {5.0, 5.0};

    EXPECT_FALSE(GeometryUtils::isLineInsidePolygon(a, b, polygon));
}

// Test 3: Line im Polygon, aber berührt die Polygonkoordinate
TEST(GeometryUtilsTest, IsLineInsidePolygon_LineInsideTouchCoord)
{
    Polygon polygon = createSquare();
    Point a = {-10.0, -10.0};
    Point b = {5.0, 5.0};

    EXPECT_FALSE(GeometryUtils::isLineInsidePolygon(a, b, polygon));
}

// Test 4: Line außerhalb vom Polygon
TEST(GeometryUtilsTest, IsLineInsidePolygon_LineOutside)
{
    Polygon polygon = createSquare();
    Point a = {-50.0, -50.0};
    Point b = {50.0, -50.0};

    EXPECT_FALSE(GeometryUtils::isLineInsidePolygon(a, b, polygon));
}

// Test 5: Line kruezt Polygon
TEST(GeometryUtilsTest, IsLineInsidePolygon_LineCross)
{
    Polygon polygon = createSquare();
    Point a = {-50.0, -50.0};
    Point b = {50.0, 50.0};

    EXPECT_FALSE(GeometryUtils::isLineInsidePolygon(a, b, polygon));
}

// ******************GeometryUtils::calculateDistance*******************

// 1.0 Berechne Distanz gröeßr 0
TEST(GeometryUtilsTest, CalculateDistance_10)
{
    double result = GeometryUtils::calculateDistance({5.0, 5.0}, {5.0, -5.0});
    EXPECT_NEAR(result, 10.0, 1e-7);
}

// 2.0 Berechne Distanz gleich 0
TEST(GeometryUtilsTest, CalculateDistance_0)
{
    double result = GeometryUtils::calculateDistance({-2.0, 5.0}, {-2.0, 5.0});
    EXPECT_NEAR(result, 0.0, 1e-7);
}

// ******************GeometryUtils::rotatePoint*******************

// 1. Test: 90 Grad Drehung gegen den Uhrzeigersinn (PI/2)
// Punkt (1, 0) rotiert um 90° sollte (0, 1) ergeben
TEST(GeometryUtilsTest, RotatePoint_90)
{
    Point p1{1.0, 0.0};
    double angle90 = M_PI / 2.0;
    Point result1 = GeometryUtils::rotatePoint(p1, angle90);

    EXPECT_NEAR(result1.x, 0.0, 1e-7);
    EXPECT_NEAR(result1.y, 1.0, 1e-7);
}

// 2. Test: 180 Grad Drehung (PI)
// Punkt (1, 2) rotiert um 180° sollte (-1, -2) ergeben
TEST(GeometryUtilsTest, RotatePoint_180)
{
    Point p2{1.0, 2.0};
    double angle180 = M_PI;
    Point result2 = GeometryUtils::rotatePoint(p2, angle180);

    EXPECT_NEAR(result2.x, -1.0, 1e-7);
    EXPECT_NEAR(result2.y, -2.0, 1e-7);
}

// 3. Test: Null-Drehung
// Punkt sollte unverändert bleiben
TEST(GeometryUtilsTest, RotatePoint_0)
{
    Point p3{5.5, -3.2};
    Point result3 = GeometryUtils::rotatePoint(p3, 0.0);

    EXPECT_DOUBLE_EQ(result3.x, 5.5);
    EXPECT_DOUBLE_EQ(result3.y, -3.2);
}

// 4. Test: Rotation des Nullpunkts (0,0)
// Der Ursprung sollte immer (0,0) bleiben, egal welcher Winkel
TEST(GeometryUtilsTest, RotatePoint_00)
{
    Point origin{0.0, 0.0};
    Point result4 = GeometryUtils::rotatePoint(origin, 1.2345);

    EXPECT_NEAR(result4.x, 0.0, 1e-7);
    EXPECT_NEAR(result4.y, 0.0, 1e-7);
}

// 5. Test: Rotiere einen Linestring
TEST(GeometryUtilsTest, RotateLineString)
{
    LineString ls;
    ls.addPoint({1, 0});
    ls.addPoint({0, 1});

    // 90 Grad Drehung
    ls.rotate(M_PI / 2.0);

    auto pts = ls.getPoints();
    ASSERT_EQ(pts.size(), 2UL);

    // (1,0) wird zu (0,1)
    EXPECT_NEAR(pts[0].x, 0.0, 1e-7);
    EXPECT_NEAR(pts[0].y, 1.0, 1e-7);

    // (0,1) wird zu (-1,0)
    EXPECT_NEAR(pts[1].x, -1.0, 1e-7);
    EXPECT_NEAR(pts[1].y, 0.0, 1e-7);
}

class GeometryUtilsTests : public ::testing::Test
{
protected:
    Polygon createSquare(bool ccw)
    {
        std::vector<Point> pts;
        if (ccw)
        {
            // Gegenuhrzeigersinn (CCW) -> Positive Fläche
            pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
        }
        else
        {
            // Uhrzeigersinn (CW) -> Negative Fläche
            pts = {{0, 0}, {0, 10}, {10, 10}, {10, 0}};
        }
        Polygon p;
        p.setPoints(pts);
        return p;
    }
};

// ******************GeometryUtils::calculatedSignedArea und ensureOrientation*******************

// 1. Test für die Berechnung der vorzeichenbehafteten Fläche
TEST_F(GeometryUtilsTests, GeometryUtilsTests_CalculateSignedArea)
{
    Polygon ccwSquare = createSquare(true);
    Polygon cwSquare = createSquare(false);

    // Ein 10x10 Quadrat hat eine Fläche von 100
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(ccwSquare), 100.0, 1e-9);
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(cwSquare), -100.0, 1e-9);
}

// 2. Test für die Sicherstellung der Orientierung (CCW erzwingen)
TEST_F(GeometryUtilsTests, GeometryUtilsTests_EnsureOrientationCCW)
{
    // 1. Setup: Starte mit einem CW Quadrat
    // Punkte: {0,0}, {0,10}, {10,10}, {10,0} -> Area -100
    Polygon poly = createSquare(false);
    ASSERT_LT(GeometryUtils::calculateSignedArea(poly), 0);

    // 2. Aktion: Erzwinger CCW
    GeometryUtils::ensureOrientation(poly, true);

    // 3. Prüfung der Fläche
    EXPECT_GT(GeometryUtils::calculateSignedArea(poly), 0);
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(poly), 100.0, 1e-9);

    // 4. Prüfung der Punkt-Reihenfolge
    auto pts = poly.getPoints();
    // Nach dem reverse von {0,0}, {0,10}, {10,10}, {10,0}
    // Erwarten wir: {10,0}, {10,10}, {0,10}, {0,0}

    EXPECT_EQ(pts[0].x, 10.0);
    EXPECT_EQ(pts[0].y, 0.0);
    EXPECT_EQ(pts[1].x, 10.0);
    EXPECT_EQ(pts[1].y, 10.0);
    EXPECT_EQ(pts[2].x, 0.0);
    EXPECT_EQ(pts[2].y, 10.0);
    EXPECT_EQ(pts[3].x, 0.0);
    EXPECT_EQ(pts[3].y, 0.0);
}

// 3. Test für die Sicherstellung der Orientierung (CW erzwingen)
TEST_F(GeometryUtilsTests, GeometryUtilsTests_EnsureOrientationCW)
{
    // Starte mit einem CCW Quadrat (falsch herum für Obstacle)
    Polygon poly = createSquare(true);
    EXPECT_GT(GeometryUtils::calculateSignedArea(poly), 0);

    // Erzwinger CW (für Obstacles)
    GeometryUtils::ensureOrientation(poly, false);

    EXPECT_LT(GeometryUtils::calculateSignedArea(poly), 0); // Check: Jetzt CW
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(poly), -100.0, 1e-9);
}