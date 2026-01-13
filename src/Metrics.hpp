#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include "Sim.hpp"

inline float minSeparation(const Sim& sim) {
    float best = std::numeric_limits<float>::infinity();
    for (int i = 0; i < (int)sim.drones.size(); ++i) {
        for (int j = i + 1; j < (int)sim.drones.size(); ++j) {
            Vec3 d = sim.drones[i].pos - sim.drones[j].pos;
            best = std::min(best, d.norm());
        }
    }
    return best;
}

inline float avgFormationError(const Sim& sim) {
    if (sim.drones.empty() || sim.slotsWorld.empty()) return 0.0f;

    float s = 0.0f;
    int count = 0;

    for (const auto& d : sim.drones) {
        if (d.slotId < 0 || d.slotId >= (int)sim.slotsWorld.size()) continue;
        s += (sim.slotsWorld[d.slotId] - d.pos).norm();
        count++;
    }
    return (count > 0) ? (s / (float)count) : 0.0f;
}
