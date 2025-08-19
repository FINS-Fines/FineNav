#pragma once

namespace TomographyConfig {
struct Params {
    float resolution = 0.1f;       // Map resolution (meters)
    float slice_dh = 0.5f;         // Height interval between slices // TODO: 它是如何影响的
    float ground_h = 0.0f;         // Ground height
    int half_kernel_size = 7;      // Traversal kernel half-size
    float interval_min = 0.50f;     // Minimum traversable interval
    float interval_free = 0.65f;    // Free space interval
    float slope_max = 0.36f;      // Maximum traversable slope (degrees)
    float step_max = 0.17f;         // Maximum step height
    float standable_ratio = 0.2f;  // Ratio of standable points required
    float cost_barrier = 50.0f;  // Cost for non-traversable areas
    float safe_margin = 0.4f;      // Safe margin around obstacles // TODO:硬安全边界?
    float inflation = 0.2f;        // Inflation radius // TODO:膨胀层？
};
}