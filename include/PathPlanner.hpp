#pragma once
#include "Environment.hpp"
#include <vector>

namespace Planner
{
    class PathPlanner
    {
    public:
        struct Weights
        {
            // Wie viel "billiger" ist der Draht im Vergleich zur Wiese? (0.1 = 10x billiger)
            static constexpr double WIRE_COST_MULTIPLIER = 0.1;

            // Motivationsfaktor: Wie sehr wird der Draht bei der Zielsuche bevorzugt?
            // 1.0 = neutral, 0.3 = starke Anziehungskraft zum Draht
            static constexpr double WIRE_HEURISTIC_BIAS = 0.65;

            // Sicherheitsabstand für Sichtprüfung (Vermeidung von numerischen Fehlern an Ecken)
            static constexpr double EPSILON_DISTANCE = 0.01;

            // Faktor für die finale Restflächen-Abdeckung (z.B. 0.5)
            static constexpr double FINAL_RING_SPACING_FACTOR = 0.85;
        };

        static bool enableDebugLogs;
        struct PlanningResult
        {
            LineString path;
            std::vector<LineString> debugLines;
            std::vector<LineString> debugLinesSec;
        };

        struct NavNode
        {
            Point pos;
            bool isWire = false;
            size_t wireIdx = 0;
        };

        struct AStarNode
        {
            Point pos;      // Die gegrafische Position
            double gCost;   // Bisherige tatsächliche Kosten vom Start weg
            double hCost;   // Heuristik: Geschätzte Kosten bis zum Ziel
            int parentIdx;  // Index des vorgängers in der closedList (-1 für Start)
            bool isWire;    // Handelt es sich um den virtual Wire Punkt
            size_t wireIdx; // Falls virtual Wire, dann welche Index

            double fCost() const { return gCost + hCost; }
        };
        struct Movement
        {
            bool allowed;
            double costMultiplier;
        };

        static std::vector<LineString> generateSlices(const Environment &env, double spacing);
        static std::vector<LineString> generateRingSlices(const Environment &env, double spacing, int maxRings = 0);
        static std::vector<LineString> filterRings (const std::vector<LineString> &rings, bool filterForObstacle);
        static PlanningResult connectSlices(const Environment &env, std::vector<LineString> &slices, Point startPos);
        static bool isPathClear(Point a, Point b, const Environment &env);

    private:
        struct BestNextSegment
        {
            int index = -1;
            bool reverse = false;
            double distance = 1e10;
            size_t entryPointIdx = 0;
            bool isPolygon = false;
        };

        static BestNextSegment findBestNext(Point currentPos, const std::vector<LineString> &slices, const std::vector<bool> &visited, const Environment &env);
        static BestNextSegment findBestNextFallback(Point currentPos, const std::vector<LineString> &slices, const std::vector<bool> &visited);
        static void addSliceToPath(LineString &path, const LineString &slice, BestNextSegment best);
        static std::vector<Point> getNavigationNodes(const Environment &env);
        static std::vector<NavNode> getExtendedNavNodes(const Environment &env);
        static std::vector<NavNode> prepareNavigationGraph(Point goal, const Environment &env);
        static size_t findBestNodeIdx(const std::vector<AStarNode> &openList);
        static Movement checkMovementRules(const AStarNode &current, const NavNode &next, const Environment &env);
        static std::vector<Point> reconstructPath(const AStarNode &goalNode, const std::vector<AStarNode> &closedList);
        static void updateOpenList(const AStarNode &current, const NavNode &nextNav, double costMultiplier, Point goal, int currentInClosedIdx, std::vector<AStarNode> &opneList, const std::vector<AStarNode> &closedList);
        static std::vector<Point> findAStarPath(Point start, Point goal, const Environment &env, std::vector<LineString> &debugLines);
        static std::vector<Point> smoothPath(const std::vector<Point> &path, const Environment &env);
    };
}