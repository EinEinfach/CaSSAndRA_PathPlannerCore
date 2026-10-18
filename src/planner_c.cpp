// C interface of the planner (include/planner_c.h). Converts the flat C types
// to the C++ classes and makes sure no exception crosses the boundary.
#include "planner_c.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "Environment.hpp"
#include "Geometry.hpp"
#include "PathPlanner.hpp"
#include "PathService.hpp"

using namespace Planner;

namespace
{
    struct InvalidInput
    {
        std::string message;
    };

    void setError(char *err, size_t errLen, const std::string &message)
    {
        if (err == nullptr || errLen == 0)
            return;
        std::snprintf(err, errLen, "%s", message.c_str());
    }

    bool finitePoint(const planner_point &p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y);
    }

    std::vector<Point> toPoints(const planner_line &line, const std::string &name, size_t minCount)
    {
        if (line.count > 0 && line.points == nullptr)
            throw InvalidInput{name + " has points but no data"};
        if (line.count < minCount)
            throw InvalidInput{name + " needs at least " + std::to_string(minCount) + " points"};
        std::vector<Point> pts;
        pts.reserve(line.count);
        for (size_t i = 0; i < line.count; ++i)
        {
            if (!finitePoint(line.points[i]))
                throw InvalidInput{name + " has a point that is not a finite number"};
            pts.push_back({line.points[i].x, line.points[i].y});
        }
        return pts;
    }

    Environment toEnvironment(const planner_environment *env)
    {
        if (env == nullptr)
            throw InvalidInput{"environment is missing"};
        Environment out(Polygon(toPoints(env->perimeter, "perimeter", 3)));
        if (env->obstacle_count > 0 && env->obstacles == nullptr)
            throw InvalidInput{"obstacles are missing"};
        for (size_t i = 0; i < env->obstacle_count; ++i)
            out.addObstacle(Polygon(toPoints(env->obstacles[i], "obstacle " + std::to_string(i), 3)));
        if (env->mow_area_count > 0 && env->mow_areas == nullptr)
            throw InvalidInput{"mow areas are missing"};
        for (size_t i = 0; i < env->mow_area_count; ++i)
            out.addMowArea(Polygon(toPoints(env->mow_areas[i], "mow area " + std::to_string(i), 3)));
        if (env->virtual_wire.count == 1)
            throw InvalidInput{"virtual wire needs 0 or at least 2 points"};
        if (env->virtual_wire.count > 0)
            out.setVirtualWire(LineString(toPoints(env->virtual_wire, "virtual wire", 2)));
        return out;
    }

    PathSettings toSettings(const planner_settings *s)
    {
        if (s == nullptr)
            throw InvalidInput{"settings are missing"};
        PathSettings out;
        switch (s->pattern)
        {
        case PLANNER_PATTERN_LINES:
            out.pattern = "lines";
            break;
        case PLANNER_PATTERN_SQUARES:
            out.pattern = "squares";
            break;
        case PLANNER_PATTERN_RINGS:
            out.pattern = "rings";
            break;
        default:
            throw InvalidInput{"unknown pattern " + std::to_string(static_cast<int>(s->pattern))};
        }
        if (!std::isfinite(s->offset) || s->offset <= 0)
            throw InvalidInput{"offset must be a positive number"};
        if (!std::isfinite(s->angle))
            throw InvalidInput{"angle must be a finite number"};
        if (!std::isfinite(s->distance_to_border) || s->distance_to_border < 0)
            throw InvalidInput{"distance to border must not be negative"};
        out.offset = s->offset;
        out.angle = s->angle;
        out.distanceToBorder = s->distance_to_border;
        out.mowArea = s->mow_area;
        out.mowBorder = s->mow_border;
        out.mowBorderCcw = s->mow_border_ccw;
        out.borderLaps = s->border_laps;
        out.mowExclusionsBorder = s->mow_exclusions_border;
        out.mowExclusionsBorderCcw = s->mow_exclusions_border_ccw;
        out.exclusionsBorderLaps = s->exclusions_border_laps;
        return out;
    }

    void fill(planner_path *out, const std::vector<Point> &pts)
    {
        if (pts.empty())
            return;
        auto *buf = static_cast<planner_point *>(std::malloc(pts.size() * sizeof(planner_point)));
        if (buf == nullptr)
            throw std::bad_alloc();
        for (size_t i = 0; i < pts.size(); ++i)
            buf[i] = {pts[i].x, pts[i].y};
        out->points = buf;
        out->count = pts.size();
    }

    // Runs f and turns every exception into a status and a message.
    template <typename F>
    planner_status guarded(planner_path *out, char *err, size_t errLen, F f)
    {
        if (out == nullptr)
        {
            setError(err, errLen, "output path is missing");
            return PLANNER_INVALID_INPUT;
        }
        *out = {nullptr, 0};
        setError(err, errLen, "");
        try
        {
            return f();
        }
        catch (const InvalidInput &e)
        {
            setError(err, errLen, e.message);
            return PLANNER_INVALID_INPUT;
        }
        catch (const std::exception &e)
        {
            planner_free_path(out);
            setError(err, errLen, std::string("planner failed: ") + e.what());
            return PLANNER_FAILED;
        }
        catch (...)
        {
            planner_free_path(out);
            setError(err, errLen, "planner failed with an unknown error");
            return PLANNER_FAILED;
        }
    }
}

extern "C"
{
    void planner_default_settings(planner_settings *out)
    {
        if (out == nullptr)
            return;
        PathSettings d;
        out->pattern = PLANNER_PATTERN_LINES;
        out->offset = d.offset;
        out->angle = d.angle;
        out->distance_to_border = d.distanceToBorder;
        out->mow_area = d.mowArea;
        out->mow_border = d.mowBorder;
        out->mow_border_ccw = d.mowBorderCcw;
        out->border_laps = d.borderLaps;
        out->mow_exclusions_border = d.mowExclusionsBorder;
        out->mow_exclusions_border_ccw = d.mowExclusionsBorderCcw;
        out->exclusions_border_laps = d.exclusionsBorderLaps;
    }

    planner_status planner_compute(const planner_environment *env, const planner_settings *settings,
                                   planner_point start, planner_path *out, char *err, size_t errLen)
    {
        return guarded(out, err, errLen, [&]
                       {
            Environment e = toEnvironment(env);
            PathSettings s = toSettings(settings);
            if (!finitePoint(start))
                throw InvalidInput{"start is not a finite point"};
            Point startPos{start.x, start.y}; // computeFullTask changes it
            PathService service;
            fill(out, service.computeFullTask(e, s, startPos).path.getPoints());
            return PLANNER_OK; });
    }

    planner_status planner_find_path(const planner_environment *env, planner_point start, planner_point goal,
                                     planner_path *out, char *err, size_t errLen)
    {
        return guarded(out, err, errLen, [&]
                       {
            Environment e = toEnvironment(env);
            if (!finitePoint(start) || !finitePoint(goal))
                throw InvalidInput{"start and goal must be finite points"};
            std::vector<Point> way = PathPlanner::findPath({start.x, start.y}, {goal.x, goal.y}, e);
            if (way.empty())
            {
                setError(err, errLen, "goal cannot be reached");
                return PLANNER_NO_PATH;
            }
            fill(out, way);
            return PLANNER_OK; });
    }

    void planner_free_path(planner_path *path)
    {
        if (path == nullptr)
            return;
        std::free(path->points);
        path->points = nullptr;
        path->count = 0;
    }

    const char *planner_version(void)
    {
        static const std::string version = PathService().getVersion();
        return version.c_str();
    }
}
