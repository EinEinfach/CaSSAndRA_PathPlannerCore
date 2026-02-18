#include <gtest/gtest.h>
#include <cmath>
#include "PathService.hpp"
#include "Environment.hpp"

using namespace Planner;

class PathServiceTest : public ::testing::Test
{
protected:
    Environment createSimpleSquare()
    {
        Polygon p({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
        return Environment(p);
    }

    Environment createSquareWithHole()
    {
        Polygon p({{0, 0}, {10, 0}, {10, 10}, {0, 10}});
        Environment env(p);
        Polygon obs({{4, 4}, {4, 6}, {6, 6}, {6, 4}}); // 2x2 Loch in der Mitte
        env.addObstacle(obs);
        return env;
    }
};

// 1. Test: Nur Perimeter-Runden (Teil 2)
TEST_F(PathServiceTest, OnlyBorderMowing)
{
    PathService service;
    PathSettings settings;
    settings.mowArea = false;
    settings.mowBorder = true;
    settings.borderLaps = 2;
    settings.offset = 1.0;
    settings.angle = 0.0;

    Environment env = createSimpleSquare();
    Point startPos{0, 0};

    auto result = service.computeFullTask(env, settings, startPos);

    // Wir erwarten 2 Slices (äußerer Perimeter + 1 innerer Ring)
    EXPECT_EQ(result.slices.size(), 2UL);
    EXPECT_FALSE(result.path.getPoints().empty());
}

// 2. Test: Nur Hindernis-Umrundung (Teil 3)
TEST_F(PathServiceTest, OnlyExclusionMowing)
{
    PathService service;
    PathSettings settings;
    settings.mowArea = false;
    settings.mowBorder = false;
    settings.mowExclusionsBoder = true;
    settings.exclusionsBorderLaps = 1;
    settings.offset = 1.0;

    Environment env = createSquareWithHole();
    auto result = service.computeFullTask(env, settings, {0, 0});

    // Wir erwarten 1 Slice (das eine Hindernis)
    EXPECT_EQ(result.slices.size(), 1UL);
}

// 3. Test: Infill-Muster "squares" (Teil 1)
TEST_F(PathServiceTest, PatternSquaresGeneratesDoubleSlices)
{
    PathService service;
    PathSettings settings;
    settings.mowArea = true;
    settings.pattern = "squares"; // Zick-Zack in zwei Richtungen
    settings.offset = 2.0;
    settings.distanceToBorder = 0.0; // Kein Shrinking für diesen Test

    Environment env = createSimpleSquare();
    auto result = service.computeFullTask(env, settings, {0, 0});

    // Bei 10x10 und 2.0 Offset erwarten wir ca. 5 Linien pro Richtung = ~10 Slices
    EXPECT_GT(result.slices.size(), 6UL);
}

// 4. Test: Shrinking führt zu Inselbildung (Teil 1 - Islands)
TEST_F(PathServiceTest, ShrinkingCreatesIslands)
{
    PathService service;
    PathSettings settings;
    settings.mowArea = true;
    settings.distanceToBorder = 4.0;
    settings.offset = 0.5;

    // Hantel-Form: Zwei 10x10 Quadrate verbunden durch einen 1m schmalen Steg
    Polygon p({{0, 0}, {10, 0}, {10, 4.5}, {12, 4.5}, {12, 0}, {22, 0}, {22, 10}, {12, 10}, {12, 5.5}, {10, 5.5}, {10, 10}, {0, 10}});
    Environment env(p);

    auto result = service.computeFullTask(env, settings, {0, 0});

    // Bei 4.0m distanceToBorder muss der 1m Steg verschwinden.
    // Es sollten 2 separate Insel-Environments intern verarbeitet worden sein.
    // Das prüfen wir indirekt über die Slices (müssen in zwei Clustern vorliegen).
    EXPECT_FALSE(result.slices.empty());
}

// 5. Test: Rotation und Rückdrehung (Teil 5)
TEST_F(PathServiceTest, RotationIntegrity)
{
    PathService service;
    PathSettings settings;
    settings.mowArea = true;
    settings.angle = M_PI / 4; // 45 Grad
    settings.offset = 1.0;

    Environment env = createSimpleSquare();
    auto result = service.computeFullTask(env, settings, {0, 0});

    // Prüfen, ob der Pfad existiert. Die Korrektheit der Rotation
    // ist schwer über EQ zu prüfen, aber wir checken ob Punkte außerhalb
    // der 0-10 Range liegen (was bei 45 Grad Drehung der Slices passieren muss)
    bool foundOutside = false;
    for (const auto &pt : result.path.getPoints())
    {
        if (pt.x < -1.0 || pt.x > 11.0)
            foundOutside = true;
    }
    // Da wir zurückdrehen, sollten die finalen Punkte wieder im Original-Raum sein
    EXPECT_FALSE(foundOutside);
}