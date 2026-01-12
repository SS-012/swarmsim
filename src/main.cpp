#include <iostream>
#include <cmath>
#include "Vec3.hpp"
#include "Formation.hpp"  

void mirrorCheck(const std::vector<Vec3>& pts, float eps=1e-3f) {
    int unmatched = 0;

    for (const auto& p : pts) {
        if (std::fabs(p.y) < eps) continue; // skip points on centerline

        bool found = false;
        for (const auto& q : pts) {
            if (std::fabs(p.x - q.x) < eps && std::fabs(p.y + q.y) < eps) {
                found = true;
                break;
            }
        }
        if (!found) {
            unmatched++;
            std::cout << "No mirror for: (" << p.x << ", " << p.y << ")\n";
        }
    }

    std::cout << "Unmatched (excluding centerline): " << unmatched << "\n";
}


int main() {
    auto v1 = makeVShape(7, 2.0f);
    mirrorCheck(v1);

    auto v2 = makeVShape(8, 2.0f, 35.0f);
    mirrorCheck(v2);

}
