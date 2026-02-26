#include <gtest/gtest.h>
#include "Environment.hpp"
#include "Geometry.hpp"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using namespace Planner;

class EnvironmentTest : public ::testing::Test
{
protected:
    // Helfer, um ein Quadrat mit definierter Wicklung zu erstellen
    Polygon createSquare(bool ccw)
    {
        std::vector<Point> pts;
        if (ccw)
        {
            pts = {{0, 0}, {10, 0}, {10, 10}, {0, 10}}; // CCW
        }
        else
        {
            pts = {{0, 0}, {0, 10}, {10, 10}, {10, 0}}; // CW
        }
        Polygon p;
        p.setPoints(pts);
        return p;
    }
};

// 1. Test: Perimeter wird beim Erstellen automatisch auf CCW normiert
TEST_F(EnvironmentTest, EnvironmentTest_PerimeterOrientationNormalization)
{
    // Wir übergeben ein CW-Quadrat (eigentlich falsch für Perimeter)
    Polygon cwPerimeter = createSquare(false);
    ASSERT_LT(GeometryUtils::calculateSignedArea(cwPerimeter), 0);

    Environment env(cwPerimeter);

    // Die Environment muss es intern in CCW umgewandelt haben
    EXPECT_GT(GeometryUtils::calculateSignedArea(env.getPerimeter()), 0);
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(env.getPerimeter()), 100.0, 1e-9);
}

// 2. Test: Obstacles werden automatisch auf CW normiert (Löcher)
TEST_F(EnvironmentTest, EnvironmentTest_ObstacleOrientationNormalization)
{
    Environment env(createSquare(true)); // Valider Perimeter

    // Wir fügen ein CCW-Obstacle hinzu (falsch, sollte CW sein)
    Polygon ccwObs = createSquare(true);
    env.addObstacle(ccwObs);

    const auto &obsList = env.getObstacles();
    ASSERT_EQ(obsList.size(), 1UL);

    // Die Environment muss das Obstacle in CW (negativ) umgewandelt haben
    EXPECT_LT(GeometryUtils::calculateSignedArea(obsList[0]), 0);
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(obsList[0]), -100.0, 1e-9);
}

// 3. Test: MowAreas werden automatisch auf CCW normiert (Inseln)
TEST_F(EnvironmentTest, EnvironmentTest_MowAreaOrientationNormalization)
{
    Environment env(createSquare(true));

    // Wir fügen eine CW-MowArea hinzu
    Polygon cwArea = createSquare(false);
    env.addMowArea(cwArea);

    const auto &mowList = env.getMowAreas();
    ASSERT_EQ(mowList.size(), 1UL);

    // Muss jetzt CCW (positiv) sein
    EXPECT_GT(GeometryUtils::calculateSignedArea(mowList[0]), 0);
    EXPECT_NEAR(GeometryUtils::calculateSignedArea(mowList[0]), 100.0, 1e-9);
}

// 4. Test: Rotation erhält die Orientierung
TEST_F(EnvironmentTest, EnvironmentTest_RotationPreservesOrientation)
{
    Environment env(createSquare(true));
    env.addObstacle(createSquare(false));

    // 90 Grad rotieren
    env.rotate(M_PI / 2.0);

    // Orientierungen müssen trotz Rotation stabil bleiben
    EXPECT_GT(GeometryUtils::calculateSignedArea(env.getPerimeter()), 0);
    EXPECT_LT(GeometryUtils::calculateSignedArea(env.getObstacles()[0]), 0);
}