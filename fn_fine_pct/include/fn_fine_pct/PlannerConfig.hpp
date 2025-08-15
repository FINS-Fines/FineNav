// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace PlannerConfig {
struct Params {
    float max_traversable_cost = 2.5f;       // Maximum cost for traversable terrain
    float height_change_weight = 2.0f;       // Weight for height changes in cost
    float layer_change_penalty = 1.0f;       // Penalty for changing layers
    float max_step_height = 0.3f;            // Maximum allowed step height
    float max_slope = 30.0f;                 // Maximum allowed slope (degrees)
    float traversal_cost_threshold = 2.5f;    // Cost threshold for traversability
};
}