#include <gtest/gtest.h>
#include "PathPlanner.hpp"
#include "Geometry.hpp"

using namespace Planner;

//***********************Unit Tests ***************************/

// ******************PathPlanner::isPathClear*******************

//Helper method to create environment
Environment getEnv() {
    Polygon perimeter = {{-5.0, -5.0}, {-5.0, -3.0}, {-3.0, -3.0}, {-3.0, 3.0}, {-5.0, 3.0}, {-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}};
    Environment env = Environment{perimeter};
    Polygon obs = {{2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
    env.addObstacle(obs);
    return env;
}

// Hinderniss
// 1. Weg schneidet Hinderniss
TEST(PathPlannerTest, IsPathClear_LineCrossObstacle) {
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({1.0, 3.0}, {5.0, 3.0}, env));
}
// 2. Weg geht über dem Obstacle
TEST(PathPlannerTest, IsPathClear_LineOverObstacle) {
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 4.5}, {4.5, 4.5}, env));
}
// 3. Weg berührt Obstacle
TEST(PathPlannerTest, IsPathClear_LineTouchObstacle) {
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 3.0}, {2.0, 3.5}, env));
}
// 4. Weg berührt Obstacle Koordinate
TEST(PathPlannerTest, IsPathClear_LineTouchObstacleCoord) {
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({1.0, 3.0}, {2.0, 2.0}, env));
}
// 5. Weg liegt im Obstacle 
TEST(PathPlannerTest, IsPathClear_LineInsideObstacle) {
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({2.5, 3.0}, {3.5, 2.5}, env));
}

// Perimeter
// 1. Weg schneidet Perimeter
TEST(PathPlannerTest, IsPathClear_LineCrossPerimeter) {
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({-4.0, -6.0}, {-3.5, 6.0}, env));
}
// 2. Weg schneidet Perimeter
TEST(PathPlannerTest, IsPathClear_LineOverPerimeter) {
    Environment env = getEnv();
    EXPECT_FALSE(PathPlanner::isPathClear({-6.0, 6.0}, {6.0, 5.0}, env));
}
// 3. Weg berührt Perimeter von innerhalb
TEST(PathPlannerTest, IsPathClear_LineTouchPerimeter) {
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({-3.0, 3.0}, {-5.0, 2.5}, env));
}
// 4. Weg berührt Perimeter-Koordinate von innerhalb
TEST(PathPlannerTest, IsPathClear_LineTouchPerimeterCoord) {
    Environment env = getEnv();
    EXPECT_TRUE(PathPlanner::isPathClear({-3.0, 3.0}, {-5.0, -5.0}, env));
}