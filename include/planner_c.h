/*
 * C interface of the path planner, for bindings that cannot use C++ directly
 * (e.g. Go via cgo). Only flat C types cross the boundary; no C++ exception
 * ever leaves these functions, errors come back as a status and a message.
 *
 * Memory: inputs are borrowed for the duration of a call. A successful call
 * fills a planner_path that the caller releases with planner_free_path.
 */
#ifndef PLANNER_C_H
#define PLANNER_C_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double x;
    double y;
} planner_point;

/* A polygon (closed or open, orientation does not matter) or a line. */
typedef struct {
    const planner_point *points;
    size_t count;
} planner_line;

typedef struct {
    planner_line perimeter;          /* at least 3 points */
    const planner_line *obstacles;   /* exclusions, may be NULL if obstacle_count is 0 */
    size_t obstacle_count;
    const planner_line *mow_areas;   /* parts to mow; none = the whole perimeter */
    size_t mow_area_count;
    planner_line virtual_wire;       /* preferred way for transits (search wire); count 0 = none */
} planner_environment;

typedef enum {
    PLANNER_PATTERN_LINES = 0,
    PLANNER_PATTERN_SQUARES = 1,
    PLANNER_PATTERN_RINGS = 2
} planner_pattern;

typedef struct {
    planner_pattern pattern;
    double offset;             /* distance between lanes in meters, > 0 */
    double angle;              /* lane direction in radians */
    double distance_to_border; /* meters kept free along the border before the area */
    bool mow_area;
    bool mow_border;
    bool mow_border_ccw;
    size_t border_laps;
    bool mow_exclusions_border;
    bool mow_exclusions_border_ccw;
    size_t exclusions_border_laps;
} planner_settings;

typedef struct {
    planner_point *points; /* owned by the planner, release with planner_free_path */
    size_t count;
} planner_path;

typedef enum {
    PLANNER_OK = 0,
    PLANNER_INVALID_INPUT = 1, /* the message says which input */
    PLANNER_FAILED = 2,        /* the planner threw; the message has its text */
    PLANNER_NO_PATH = 3        /* find_path: the goal cannot be reached */
} planner_status;

/* The defaults of Planner::PathSettings. */
void planner_default_settings(planner_settings *out);

/*
 * Computes the coverage path (Planner::PathService::computeFullTask) starting
 * at start. On error, a message of at most err_len - 1 bytes is written to err
 * (if err is not NULL) and *out is left empty.
 */
planner_status planner_compute(const planner_environment *env, const planner_settings *settings,
                               planner_point start, planner_path *out, char *err, size_t err_len);

/*
 * Finds a way from start to goal around obstacles and inside the perimeter
 * (A*, preferring the virtual wire), e.g. for goto. The path starts at start
 * and ends at goal.
 */
planner_status planner_find_path(const planner_environment *env, planner_point start, planner_point goal,
                                 planner_path *out, char *err, size_t err_len);

/* Releases the points of a path filled by the planner; safe on an empty path. */
void planner_free_path(planner_path *path);

/* Version of the planner core, e.g. "1.4". The string is static. */
const char *planner_version(void);

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_C_H */
