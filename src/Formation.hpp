#pragma once
#include <vector>
#include "Vec3.hpp"

std::vector<Vec3> makeCircle(int N, float radius);
std::vector<Vec3> makeGrid(int N, float spacing);
std::vector<Vec3> makeVShape(int N, float spacing, float angleDeg = 35.0f);
