#ifndef IISPTSCHEDULEMONITOR_H
#define IISPTSCHEDULEMONITOR_H

#include <mutex>
#include "geometry.h"

namespace pbrt {

// ============================================================================
// End points are assumed to be exclusive
struct IisptScheduleMonitorTask
{
    int x;
    int y;
    int distance;
};

// ============================================================================
class IisptScheduleMonitor
{
private:

    // ------------------------------------------------------------------------
    // Members

    // Film bounds
    Bounds2i bounds;

    float current_radius;

    float update_multiplier;

    // Reset point for samples counter
    int update_interval;

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
