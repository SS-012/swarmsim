#pragma once
#include <vector>
#include "Vec3.hpp" 

std::vector<int> assignHungarianSlots(
    const std::vector<Vec3>& dronePositions,
    const std::vector<Vec3>& slotPositions
);