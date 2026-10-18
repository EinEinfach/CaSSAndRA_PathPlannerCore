#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "planner_c.h"
#include "Environment.hpp"
#include "PathPlanner.hpp"
#include "PathService.hpp"

namespace
{
    // A 10 x 10 garden with a 2 x 2 exclusion in the middle.
    const std::vector<planner_point> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const std::vector<planner_point> hole = {{4, 4}, {6, 4}, {6, 6}, {4, 6}};

    planner_line line(const std::vector<planner_point> &pts)
    {
        return {pts.data(), pts.size()};
    }

    planner_environment garden(const std::vector<planner_line> &obstacles)
    {
        planner_environment env{};
        env.perimeter = line(square);
        env.obstacles = obstacles.data();
        env.obstacle_count = obstacles.size();
        return env;
    }

    planner_settings lanes(double offset)
    {
        planner_settings s;
        planner_default_settings(&s);
        s.offset = offset;
        s.angle = 0;
        s.distance_to_border = 0;
        s.mow_border = false;
        s.mow_exclusions_border = false;
        return s;
    }

    bool inside(const planner_point &p, double minX, double minY, double maxX, double maxY, double tol = 1e-6)
    {
        return p.x >= minX - tol && p.x <= maxX + tol && p.y >= minY - tol && p.y <= maxY + tol;
    }
}

TEST(PlannerC, ComputesACoveragePathInsideThePerimeter)
{
    planner_environment env = garden({});
    planner_settings s = lanes(1.0);
    planner_path path{};
    char err[128];

    ASSERT_EQ(planner_compute(&env, &s, {0.5, 0.5}, &path, err, sizeof err), PLANNER_OK) << err;
    ASSERT_GT(path.count, 10u);
    for (size_t i = 0; i < path.count; ++i)
        EXPECT_TRUE(inside(path.points[i], 0, 0, 10, 10)) << "point " << i;
    EXPECT_STREQ(err, "");
    planner_free_path(&path);
    EXPECT_EQ(path.points, nullptr);
    EXPECT_EQ(path.count, 0u);
}

TEST(PlannerC, KeepsTheLanesOutOfExclusions)
{
    std::vector<planner_line> obstacles = {line(hole)};
    planner_environment env = garden(obstacles);
    planner_settings s = lanes(0.5);
    planner_path path{};

    ASSERT_EQ(planner_compute(&env, &s, {0.5, 0.5}, &path, nullptr, 0), PLANNER_OK);
    Planner::Environment cpp(Planner::Polygon({{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
    cpp.addObstacle(Planner::Polygon({{4, 4}, {6, 4}, {6, 6}, {4, 6}}));
    for (size_t i = 1; i < path.count; ++i)
    {
        Planner::Point a{path.points[i - 1].x, path.points[i - 1].y};
        Planner::Point b{path.points[i].x, path.points[i].y};
        EXPECT_TRUE(Planner::PathPlanner::isPathClear(a, b, cpp)) << "segment " << i;
    }
    planner_free_path(&path);
}

TEST(PlannerC, MowsOnlyTheMowAreas)
{
    const std::vector<planner_point> area = {{1, 1}, {3, 1}, {3, 3}, {1, 3}};
    std::vector<planner_line> areas = {line(area)};
    for (double distance : {0.0, 0.5})
    {
        planner_environment env = garden({});
        env.mow_areas = areas.data();
        env.mow_area_count = areas.size();
        planner_settings s = lanes(0.25);
        s.distance_to_border = distance;
        planner_path path{};

        ASSERT_EQ(planner_compute(&env, &s, {2, 2}, &path, nullptr, 0), PLANNER_OK);
        ASSERT_GT(path.count, 0u);
        size_t outside = 0;
        for (size_t i = 0; i < path.count; ++i)
            outside += !inside(path.points[i], 1, 1, 3, 3);
        EXPECT_EQ(outside, 0u) << "distance to border " << distance << ": "
                               << outside << " of " << path.count << " points outside the mow area";
        planner_free_path(&path);
    }
}

TEST(PlannerC, RejectsInvalidInputWithAMessage)
{
    const std::vector<planner_point> twoPoints = {{0, 0}, {1, 1}};
    const std::vector<planner_point> notANumber = {{0, 0}, {NAN, 1}, {1, 0}};
    const std::vector<planner_point> onePoint = {{1, 1}};

    struct Case
    {
        std::string name;
        planner_environment env;
        planner_settings settings;
        std::string want;
    };
    std::vector<Case> cases;
    auto add = [&](std::string name, auto changeEnv, auto changeSettings, std::string want)
    {
        Case c{name, garden({}), lanes(1.0), want};
        changeEnv(c.env);
        changeSettings(c.settings);
        cases.push_back(c);
    };
    auto keepEnv = [](planner_environment &) {};
    auto keepSettings = [](planner_settings &) {};
    add("perimeter too short", [&](planner_environment &e) { e.perimeter = line(twoPoints); }, keepSettings, "perimeter");
    add("perimeter NaN", [&](planner_environment &e) { e.perimeter = line(notANumber); }, keepSettings, "finite");
    add("obstacles NULL", [](planner_environment &e) { e.obstacle_count = 1; e.obstacles = nullptr; }, keepSettings, "obstacles");
    add("wire of one point", [&](planner_environment &e) { e.virtual_wire = line(onePoint); }, keepSettings, "virtual wire");
    add("zero offset", keepEnv, [](planner_settings &s) { s.offset = 0; }, "offset");
    add("NaN angle", keepEnv, [](planner_settings &s) { s.angle = NAN; }, "angle");
    add("negative distance", keepEnv, [](planner_settings &s) { s.distance_to_border = -1; }, "distance");
    add("unknown pattern", keepEnv, [](planner_settings &s) { s.pattern = static_cast<planner_pattern>(7); }, "pattern");

    for (auto &c : cases)
    {
        planner_path path{};
        char err[128] = "unchanged";
        EXPECT_EQ(planner_compute(&c.env, &c.settings, {1, 1}, &path, err, sizeof err), PLANNER_INVALID_INPUT) << c.name;
        EXPECT_NE(std::string(err).find(c.want), std::string::npos) << c.name << ": " << err;
        EXPECT_EQ(path.points, nullptr) << c.name;
    }

    planner_environment env = garden({});
    planner_settings s = lanes(1.0);
    EXPECT_EQ(planner_compute(&env, &s, {1, 1}, nullptr, nullptr, 0), PLANNER_INVALID_INPUT);
    EXPECT_EQ(planner_compute(nullptr, &s, {1, 1}, nullptr, nullptr, 0), PLANNER_INVALID_INPUT);
}

TEST(PlannerC, TruncatesLongMessages)
{
    planner_environment env = garden({});
    planner_settings s = lanes(-1);
    planner_path path{};
    char err[8];
    EXPECT_EQ(planner_compute(&env, &s, {1, 1}, &path, err, sizeof err), PLANNER_INVALID_INPUT);
    EXPECT_STREQ(err, "offset ");
}

TEST(PlannerC, FindsAWayAroundAnExclusion)
{
    std::vector<planner_line> obstacles = {line(hole)};
    planner_environment env = garden(obstacles);
    planner_path path{};
    char err[128];

    ASSERT_EQ(planner_find_path(&env, {1, 5}, {9, 5}, &path, err, sizeof err), PLANNER_OK) << err;
    ASSERT_GE(path.count, 3u); // the straight line crosses the exclusion
    EXPECT_DOUBLE_EQ(path.points[0].x, 1);
    EXPECT_DOUBLE_EQ(path.points[0].y, 5);
    EXPECT_DOUBLE_EQ(path.points[path.count - 1].x, 9);
    EXPECT_DOUBLE_EQ(path.points[path.count - 1].y, 5);
    Planner::Environment cpp(Planner::Polygon({{0, 0}, {10, 0}, {10, 10}, {0, 10}}));
    cpp.addObstacle(Planner::Polygon({{4, 4}, {6, 4}, {6, 6}, {4, 6}}));
    for (size_t i = 1; i < path.count; ++i)
    {
        Planner::Point a{path.points[i - 1].x, path.points[i - 1].y};
        Planner::Point b{path.points[i].x, path.points[i].y};
        EXPECT_TRUE(Planner::PathPlanner::isPathClear(a, b, cpp)) << "segment " << i;
    }
    planner_free_path(&path);
}

TEST(PlannerC, ReportsAnUnreachableGoal)
{
    planner_environment env = garden({});
    planner_path path{};
    char err[128];
    EXPECT_EQ(planner_find_path(&env, {1, 1}, {20, 20}, &path, err, sizeof err), PLANNER_NO_PATH);
    EXPECT_STREQ(err, "goal cannot be reached");
    EXPECT_EQ(path.count, 0u);
}

TEST(PlannerC, FreeingAnEmptyPathIsSafe)
{
    planner_path path{};
    planner_free_path(&path);
    planner_free_path(nullptr);
    SUCCEED();
}

TEST(PlannerC, ReportsItsVersion)
{
    EXPECT_STREQ(planner_version(), "1.4");
}

TEST(PlannerC, DefaultSettingsMatchTheCppDefaults)
{
    planner_settings s;
    planner_default_settings(&s);
    Planner::PathSettings d;
    EXPECT_EQ(s.pattern, PLANNER_PATTERN_LINES);
    EXPECT_EQ(s.offset, d.offset);
    EXPECT_EQ(s.distance_to_border, d.distanceToBorder);
    EXPECT_EQ(s.border_laps, d.borderLaps);
    EXPECT_EQ(s.mow_exclusions_border, d.mowExclusionsBorder);
    EXPECT_EQ(s.exclusions_border_laps, d.exclusionsBorderLaps);
}
