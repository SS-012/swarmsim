#include <cmath>
#include <vector>
#include "Vec3.hpp"
#include "Formation.hpp"

std::vector<Vec3> makeCircle(int N, float radius){
    std::vector<Vec3> points;
    for (int i = 0; i < N; i++){
        float angle = 2 * M_PI * i / N;
        points.push_back(Vec3(radius * cos(angle), radius * sin(angle), 0));
    }
    return points;
}

std::vector<Vec3> makeGrid(int N, float spacing) {
    std::vector<Vec3> points;
    int rows = static_cast<int>(std::sqrt(N));
    int cols = (N + rows - 1) / rows;  // Ceiling division
    for (int i = 0; i < N; i++){
        int row = i / cols;
        int col = i % cols;
        points.push_back(Vec3(col * spacing, row * spacing, 0));
    }
    return points;
}

std::vector<Vec3> makeVShape(int N, float spacing, float angleDeg) {
    std::vector<Vec3> points;
    if (N <= 0) return points;

    float angleRad = angleDeg * M_PI / 180.0f; 

    points.emplace_back(0.0f, 0.0f, 0.0f);

    int remaining = N - 1;
    int leftCount = (remaining + 1) / 2;
    int rightCount = (remaining / 2);

    Vec3 leftDir(-std::cos(angleRad), std::sin(angleRad), 0.0f);
    Vec3 rightDir(-std::cos(angleRad), -std::sin(angleRad), 0.0f);

    for (int i = 1; i <= leftCount; i++) {
        points.push_back(leftDir * (i * spacing));
    }
    for (int i = 1; i <= rightCount; i++) {
        points.push_back(rightDir * (i * spacing));
    }

    Vec3 centroid(0.0f, 0.0f, 0.0f);

    for (const auto& p : points) centroid += p;
    centroid /= static_cast<float>(points.size());

    for (auto& p : points) p -= centroid;

    return points;
}


