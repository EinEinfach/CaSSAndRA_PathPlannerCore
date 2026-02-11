#pragma once
#include <vector>
#include <initializer_list>

namespace Planner
{

    struct Point
    {
        double x;
        double y;
    };

    class LineString
    {
    public:
        LineString() = default;
        LineString(std::initializer_list<Point> pts); 

        void addPoint(Point p);

        const std::vector<Point>& getPoints() const;

    protected:
        std::vector<Point> points;
    };

    class Polygon : public LineString
    {
    public:
        using LineString::LineString;
        bool isClosed() const;
    };
};