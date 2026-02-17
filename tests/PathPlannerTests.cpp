#define private public
#include <gtest/gtest.h>
#include "PathPlanner.hpp"
#include "Geometry.hpp"
#undef private

using namespace Planner;

//***********************Unit Tests ***************************/

// ******************PathPlanner::isPathClear*******************

// Helper method to create environment
Environment getEnv()
{
    Polygon perimeter = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    Environment env = Environment{perimeter};
    Polygon obs = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    env.addObstacle(obs);
    return env;
}

// Hinderniss
// 1. Weg schneidet Hinderniss
TEST(PathPlannerTest, IsPathClear_LineCrossObstacle)
{
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({1.0, 3.0}, {5.0, 3.0}, env));
}
// 2. Weg geht über dem Obstacle
TEST(PathPlannerTest, IsPathClear_LineOverObstacle)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 4.5}, {4.5, 4.5}, env));
}
// 3. Weg berührt Obstacle
TEST(PathPlannerTest, IsPathClear_LineTouchObstacle)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 3.0}, {2.0, 3.5}, env));
}
// 4. Weg berührt Obstacle Koordinate
TEST(PathPlannerTest, IsPathClear_LineTouchObstacleCoord)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 3.0}, {2.0, 2.0}, env));
}
// 5. Weg liegt im Obstacle
TEST(PathPlannerTest, IsPathClear_LineInsideObstacle)
{
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({2.5, 3.0}, {3.5, 2.5}, env));
}

// Perimeter
// 1. Weg schneidet Perimeter
TEST(PathPlannerTest, IsPathClear_LineCrossPerimeter)
{
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({-4.0, -6.0}, {-3.5, 6.0}, env));
}
// 2. Weg schneidet Perimeter
TEST(PathPlannerTest, IsPathClear_LineOverPerimeter)
{
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({-6.0, 6.0}, {6.0, 5.0}, env));
}
// 3. Weg berührt Perimeter von innerhalb
TEST(PathPlannerTest, IsPathClear_LineTouchPerimeter)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({-3.0, 3.0}, {-3.0, 5.0}, env));
}
// 4. Weg berührt Perimeter-Koordinate von innerhalb
TEST(PathPlannerTest, IsPathClear_LineTouchPerimeterCoord)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({-3.0, 3.0}, {-3.0, 4.0}, env));
}
// 5. Weg start und ende innerhalb des Perimeters aber mindestens ein Punkt des Weges außerhalb
TEST(PathPlannerTest, IsPathClear_LineOutsideConcavePart)
{
    Environment env = getEnv();
    // Start und Ziel sind im Perimeter, aber die direkte Linie würde "durch die Luft"
    // außerhalb des U-Ausschnitts gehen (z.B. von (-4, -4) nach (-4, 4))
    EXPECT_FALSE(PathPlanner::isPathClear({-4.0, -4.0}, {-4.0, 4.0}, env));
}
// 6. Extrem kurzer Weg (kleiner als Epsilon)
TEST(PathPlannerTest, IsPathClear_VeryShortLine)
{
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({0.0, 0.0}, {0.00001, 0.00001}, env));
}

// ******************PathPlanner::generateSlices*******************

// 1. Prüft, ob ein Hindernis einen Slice korrekt in zwei Teile zerlegt
TEST(PathPlannerTest, GenerateSlices_SliceCutByPerimeter)
{
    Polygon perimeter = {{-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}};
    Environment env = Environment(perimeter);
    PathPlanner planner;
    double spacing = 1.0;
    auto slices = planner.generateSlices(env, spacing);

    for (auto &slice : slices)
    {
        auto &points = slice.getPoints();
        EXPECT_NEAR(points[0].x, -1.0, 1e-7);
        EXPECT_NEAR(points[1].x, 1.0, 1e-7);
    }
}

// 2. Prüft, ob ein Hindernis einen Slice korrekt in zwei Teile zerlegt
TEST(PathPlannerTest, GenerateSlices_ObstacleSplitsSlice)
{
    Environment env = getEnv();
    PathPlanner planner;
    double spacing = 1.0;

    auto slices = planner.generateSlices(env, spacing);

    // Suche Slices auf der Höhe y=3.0 (mitten durch das Hindernis)
    int slicesAtY3 = 0;
    for (const auto &slice : slices)
    {
        if (std::abs(slice.getPoints()[0].y - 3.0) < 1e-7)
        {
            slicesAtY3++;
            // Sicherstellen, dass kein Punkt im Hindernis liegt (x zwischen 2 und 4)
            for (const auto &p : slice.getPoints())
            {
                EXPECT_FALSE(p.x > 2.1 && p.x < 3.9);
            }
        }
    }
    // Erwartung: Mindestens zwei Slices bei y=3 (links vom Hindernis und rechts davon)
    EXPECT_GE(slicesAtY3, 2);
}

// 3. Slice trifft genau auf die Kante eines Hindernisses
TEST(PathPlannerTest, GenerateSlices_EdgeCaseTangent)
{
    Environment env = getEnv();
    PathPlanner planner;
    // Spacing so wählen, dass y exakt 2.0 trifft (Unterkante Obstacle)
    // minY ist -5.0. spacing = 7.0 führt zu y = 2.0
    auto slices = planner.generateSlices(env, 7.0);

    for (const auto &slice : slices)
    {
        // Der Pfad darf das Hindernis berühren, aber nicht hindurchgehen
        // Hier prüfen wir nur auf Absturz oder "Unendlich-Schleifen"
        EXPECT_NO_THROW(slice.getPoints());
    }
}

// 4. Slice liegt zum Teil auf der Kante des Perimeters
TEST(PathPlannerTest, GenerateSlices_SliceOnPerimeterEdge)
{
    Environment env = getEnv();
    PathPlanner planner;
    double spacing = 1.0;

    auto slices = planner.generateSlices(env, spacing);
    LineString &edgeSlice = slices[1];
    LineString &roomSlice = slices[2];

    EXPECT_NEAR(edgeSlice.getPoints()[0].x, -5.0, 1e-7);
    EXPECT_NEAR(edgeSlice.getPoints()[1].x, -3.0, 1e-7);

    EXPECT_NEAR(roomSlice.getPoints()[0].x, -3.0, 1e-7);
    EXPECT_NEAR(roomSlice.getPoints()[1].x, 5.0, 1e-7);
}

// 5. Slice geht durch die Perimeter Koordinate
TEST(PathPlannerTest, GenerateSlices_SliceThroughPerimeterCoord)
{
    Polygon perimeter = {{-1.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {0.0, -1.0}};
    Environment env = Environment(perimeter);
    PathPlanner planner;
    double spacing = 1.0;

    auto slices = planner.generateSlices(env, spacing);
    LineString &slice = slices[0];

    ASSERT_GE(slices.size(), 1UL);
    EXPECT_NEAR(slice.getPoints()[0].x, -1.0, 1e-7);
    EXPECT_NEAR(slice.getPoints()[1].x, 1.0, 1e-7);
}

// ******************PathPlanner::connectSlices*******************

// 1. Testet, ob Slices in der richtigen Reihenfolge verbunden werden (Nächster-Punkt-Logik)
TEST(PathPlannerTest, ConnectSlices_Ordering)
{
    Environment env = getEnv();
    PathPlanner planner;

    std::vector<LineString> slices;
    // Zwei parallele Slices manuell erstellen
    LineString s1, s2;
    s1.addPoint({-2.0, 0.0});
    s1.addPoint({2.0, 0.0});
    s2.addPoint({-2.0, 1.0});
    s2.addPoint({2.0, 1.0});
    slices.push_back(s1);
    slices.push_back(s2);

    // Start bei (2.1, 0.0) -> s1 sollte von rechts nach links (reverse) genommen werden
    auto result = planner.connectSlices(env, slices, {2.1, 0.0});

    ASSERT_GE(result.path.getPoints().size(), 4UL);
    // Erster Punkt nach Start sollte Ende von s1 sein
    EXPECT_NEAR(result.path.getPoints()[1].x, 2.0, 1e-7);
}

// 2. Testet, ob Slices in der richtigen Reihenfolge verbunden werden (Slices zwischen Obstacle und Perimeter)
TEST(PathPlannerTest, ConnectSlices_OrderingBetweenObsPer)
{
    Environment env = getEnv();
    PathPlanner planner;

    std::vector<LineString> slices;
    // Zwei parallele Slices manuell erstellen
    LineString s1;
    s1.addPoint({4.0, 2.0});
    s1.addPoint({5.0, 2.0});
    slices.push_back(s1);

    // Start bei (4.0, 3.0) -> s1 sollte von rechts nach links (forward) genommen werden
    auto result = planner.connectSlices(env, slices, {4.0, 3.0});

    // Erster Punkt nach Start sollte Anfang von s1 sein
    EXPECT_NEAR(result.path.getPoints()[1].x, 4.0, 1e-7);
    // Letzter Punkt im Path sollte Ende von s1 sein
    EXPECT_NEAR(result.path.getPoints()[2].x, 5.0, 1e-7);
}
// ******************PathPlanner::findAStarPath*******************

// 1. Direkter Weg ohne Hindernisse
TEST(PathPlannerTest, FindAStarPath_PathStraight)
{
    Environment env = getEnv(); // Nutzt dein Standard-U-Profil
    PathPlanner planner;
    Point start = {0.0, 0.0};
    Point goal = {1.0, 0.0};

    auto path = planner.findAStarPath(start, goal, env);

    ASSERT_GE(path.size(), 2UL);
    EXPECT_NEAR(path.front().x, 0.0, 1e-3);
    EXPECT_NEAR(path.back().x, 1.0, 1e-3);
}

// 2. Weg meidet Hindernisse
TEST(PathPlannerTest, FindAStarPath_PathAroundObstacle)
{
    Environment env = getEnv();
    PathPlanner planner;
    Point start = {1.0, 3.0}; // Links vom Hindernis
    Point goal = {5.0, 3.0};  // Rechts vom Hindernis

    auto path = planner.findAStarPath(start, goal, env);

    ASSERT_GT(path.size(), 2UL); // Muss mindestens einen Zwischenpunkt haben
    for (const auto &p : path)
    {
        // Keiner der Pfadpunkte darf im Hindernis liegen
        bool inside = (p.x > 2.01 && p.x < 3.99 && p.y > 2.01 && p.y < 3.99);
        EXPECT_FALSE(inside);
    }
}

// 3. Weg nicht möglich
TEST(PathPlannerTest, FindAStarPath_NoPathPossible)
{
    Environment env({{-10, -10}, {10, -10}, {10, 10}, {-10, 10}});

    PathPlanner planner;
    Point start = {0.0, 0.0};
    Point goal = {20.0, 20.0}; // goal outside of perimeter

    auto path = planner.findAStarPath(start, goal, env);

    EXPECT_TRUE(path.empty()); // Erwartung: Leerer Vektor
}

// 4. Start und Ziel identisch
TEST(PathPlannerTest, FindAStarPath_StartEqualsGoal)
{
    Environment env = getEnv();
    PathPlanner planner;
    Point start = {1.0, 1.0};

    auto path = planner.findAStarPath(start, start, env);

    ASSERT_EQ(path.size(), 1UL);
    EXPECT_NEAR(path[0].x, 1.0, 1e-3);
}

// 5. Komplexer Pfad
TEST(PathPlannerTest, FindAStarPath_ConcavePerimeterNavigation)
{
    Environment env = getEnv();
    PathPlanner planner;
    // Start im oberen Arm des U, Ziel im unteren Arm
    Point start = {-4.0, 4.0};
    Point goal = {-4.0, -4.0};

    auto path = planner.findAStarPath(start, goal, env);

    ASSERT_GT(path.size(), 2UL);
    // Prüfe, ob alle Punkte innerhalb des Perimeters liegen
    for (const auto &p : path)
    {
        EXPECT_TRUE(GeometryUtils::isPointCoveredByPolygon(p, env.getPerimeter()));
    }
}

// 6. Virtual Wire
TEST(PathPlannerTest, FindAStarPath_VirtualWireNavigation)
{
    // Perimeter: 0,0 bis 20,20
    Environment env = Environment({{0, 0}, {20, 0}, {20, 20}, {0, 20}});

    // Virtual Wire: Eine "Schiene" von (2,2) bis (18,2)
    LineString wire;
    wire.addPoint({2, 2});  // Index 0
    wire.addPoint({10, 2}); // Index 1
    wire.addPoint({18, 2}); // Index 2
    env.setVirtualWire(wire);

    // Hindernis, das ALLES blockiert (von y=-5 bis y+5),
    // außer die Sonderregel für den Draht.
    Polygon obs;
    obs.addPoint({5, -5}); // Tief genug, um y=0 zu blockieren
    obs.addPoint({6, -5});
    obs.addPoint({6, 5}); // Hoch genug, um y=2 theoretisch zu blockieren
    obs.addPoint({5, 5});
    env.addObstacle(obs);

    // Start und Ziel liegen auf y=0
    Point start{2, 0};
    Point goal{18, 0};

    auto path = PathPlanner::findAStarPath(start, goal, env);

    // PRÜFUNGEN:
    ASSERT_FALSE(path.empty()) << "A* sollte einen Weg über den Draht finden, da dieser das Hindernis ignoriert.";

    // Check: Wurde der Draht-Mittelpunkt genutzt?
    bool usedWireMidPoint = false;
    for (const auto &p : path)
    {
        if (std::abs(p.x - 10.0) < 0.1 && std::abs(p.y - 2.0) < 0.1)
            usedWireMidPoint = true;
    }

    EXPECT_TRUE(usedWireMidPoint) << "Der A* muss den Draht-Punkt (10,2) nutzen, da alle anderen Wege durch das Hindernis blockiert sind.";
}