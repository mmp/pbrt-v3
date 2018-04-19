#ifndef IISPTSCHEDULEMONITOR_H
#define IISPTSCHEDULEMONITOR_H

#include <mutex>
#include "geometry.h"

namespace pbrt {

// ============================================================================
// End points are assumed to be exclusive
struct IisptScheduleMonitorTask
{
    int x0;
    int y0;
    int x1;
    int y1;
    int tilesize;
};

// ============================================================================
class IisptScheduleMonitor
{
private:

    // ------------------------------------------------------------------------
    // Members

    // Number of tiles per side in each task
    int NUMBER_TILES = 10;

    // Size of a tile
    float current_radius;

    float update_multiplier;

    // Reset point for samples counter
    int update_interval;

    // Film bounds
    Bounds2i bounds;

    // Current pixels in the film
    int nextx;
    int nexty;

public:

    // Constructor ------------------------------------------------------------
    IisptScheduleMonitor(Bounds2i bounds);

    // Public methods ---------------------------------------------------------

    IisptScheduleMonitorTask next_task();

};

} // namespace pbrt

#endif // IISPTSCHEDULEMONITOR_H
