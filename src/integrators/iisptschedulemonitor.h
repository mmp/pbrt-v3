#ifndef IISPTSCHEDULEMONITOR_H
#define IISPTSCHEDULEMONITOR_H

#include <mutex>

namespace pbrt {

// ============================================================================
class IisptScheduleMonitor
{
private:

    // ------------------------------------------------------------------------
    // Members

    std::recursive_mutex lock;

    float current_radius;

    float update_multiplier;

    // Reset point for samples counter
    int update_interval;

    // Partial counter, downward
    int samples_count;

public:

    // Constructor ------------------------------------------------------------
    IisptScheduleMonitor();

    // Public methods ---------------------------------------------------------

    float get_current_radius();

};

} // namespace pbrt

#endif // IISPTSCHEDULEMONITOR_H
