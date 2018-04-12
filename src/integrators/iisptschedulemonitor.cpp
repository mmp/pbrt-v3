#include "iisptschedulemonitor.h"

#include <cstdlib>
#include <string>

namespace pbrt {

// ============================================================================
IisptScheduleMonitor::IisptScheduleMonitor() {
    // Read environment variables
    char* radius_start_env = std::getenv("IISPT_SCHEDULE_RADIUS_START");
    if (radius_start_env == NULL) {
        current_radius = 50.0;
    } else {
        current_radius = std::stof(std::string(radius_start_env));
    }

    char* radius_ratio_env = std::getenv("IISPT_SCHEDULE_RADIUS_RATIO");
    if (radius_ratio_env == NULL) {
        update_multiplier = 0.90;
    } else {
        update_multiplier = std::stof(std::string(radius_ratio_env));
    }

    char* update_interval_env = std::getenv("IISPT_SCHEDULE_INTERVAL");
    if (update_interval_env == NULL) {
        update_interval = 500;
    } else {
        update_interval = std::stoi(std::string(update_interval_env));
    }

    // Initialize samples count
    samples_count = update_interval;
}

// ============================================================================
float IisptScheduleMonitor::get_current_radius() {
    lock.lock();

    // Decrement samples count
    samples_count--;
    if (samples_count <= 0) {
        // Update radius
        current_radius = current_radius * update_multiplier;
        samples_count = update_interval;
    }

    // Return radius
    lock.unlock();
    return current_radius;
}

}
