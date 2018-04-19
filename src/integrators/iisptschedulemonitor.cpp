#include "iisptschedulemonitor.h"

#include <cstdlib>
#include <string>

namespace pbrt {

// ============================================================================
IisptScheduleMonitor::IisptScheduleMonitor(Bounds2i bounds) {
    this->bounds = bounds;

    // Read environment variables
    char* radius_start_env = std::getenv("IISPT_SCHEDULE_RADIUS_START");
    if (radius_start_env == NULL) {
        current_radius = 50.0;
    } else {
        current_radius = std::stof(std::string(radius_start_env));
    }

    char* radius_ratio_env = std::getenv("IISPT_SCHEDULE_RADIUS_RATIO");
    if (radius_ratio_env == NULL) {
        update_multiplier = 0.50;
    } else {
        update_multiplier = std::stof(std::string(radius_ratio_env));
    }

    char* update_interval_env = std::getenv("IISPT_SCHEDULE_INTERVAL");
    if (update_interval_env == NULL) {
        update_interval = 250;
    } else {
        update_interval = std::stoi(std::string(update_interval_env));
    }

    nextx = bounds.pMin.x;
    nexty = bounds.pMin.y;
}

// ============================================================================
IisptScheduleMonitorTask IisptScheduleMonitor::next_task() {

    int effective_radius = std::floor(current_radius);
    if (effective_radius < 1) {
        effective_radius = 1;
    }

    // Form the result
    // The current nextx and nexty are valid starting coordinates
    IisptScheduleMonitorTask res;
    res.x = nextx;
    res.y = nexty;
    res.distance = effective_radius;

    // Advance to the next tile

    nextx += effective_radius;
    if (nextx >= bounds.pMax.x) {
        // Reset x, advance y
        nextx = bounds.pMin.x;
        nexty += effective_radius;
    }

    if (nexty >= bounds.pMax.y) {
        // Reset y, advance radius
        nexty = bounds.pMin.y;
        current_radius *= update_multiplier;
    }

    return res;
}

}
