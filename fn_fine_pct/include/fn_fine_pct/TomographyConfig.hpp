#pragma once

namespace TomographyConfig {
struct Params {
    float resolution = 0.1f;       // Map resolution (meters)
    float slice_dh = 0.2f;         // Height interval between slices
    float ground_h = 0.0f;         // Ground height
    int half_kernel_size = 2;      // Traversal kernel half-size
    float interval_min = 0.1f;     // Minimum traversable interval
    float interval_free = 0.5f;    // Free space interval
    float slope_max = 30.0f;      // Maximum traversable slope (degrees)
    float step_max = 0.3f;         // Maximum step height
    float standable_ratio = 0.5f;  // Ratio of standable points required
    float cost_barrier = 1000.0f;  // Cost for non-traversable areas
    float safe_margin = 0.2f;      // Safe margin around obstacles
    float inflation = 0.3f;        // Inflation radius
};
}