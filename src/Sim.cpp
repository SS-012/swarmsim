#include "Sim.hpp" 
#include "Utils.hpp"
#include <cmath>

void Sim::ensureBuffers() {
    int N = (int)drones.size();
    if ((int)posCache_.size() != N) posCache_.resize(N);

    if (scratch_.capacity() < 128) scratch_.reserve(128);
}

void Sim::step() {
    if (slotsWorld.size() != drones.size()) return;

    ensureBuffers();

    const size_t n = drones.size();

    for (size_t i = 0; i < n; ++i) {
        posCache_[i] = drones[i].pos;
    }

    grid_.setCellSize(params_.sepParams.neighborR);
    grid_.build(posCache_);

    for (int i = 0; i < (int)n; ++i) {
        auto& d = drones[i];
        if (d.slotId < 0 || d.slotId >= (int)slotsWorld.size()) continue;

        const Vec3& target = d.useTempTarget ? d.tempTarget : slotsWorld[d.slotId];

        Vec3 a_form = PDFormation(d, target, params_.pdParams);
        Vec3 a_sep  = separationAccelHashed(i, posCache_, params_.sepParams, grid_, scratch_);

        Vec3 a = a_form + a_sep;  
        d.vel += a * params_.dt;
        d.vel = clampNorm(d.vel, params_.vMax);
        d.pos += d.vel * params_.dt;
    }
}
