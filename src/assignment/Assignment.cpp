#include "Assignment.hpp"
#include <limits>
#include <algorithm>
#include "Utils.hpp"


static std::vector<int> hungarianMin(
    const std::vector<std::vector<float>>& costMatrix
) {
    int n = (int)costMatrix.size();
    int m = (int)costMatrix[0].size();

    const float INF = std::numeric_limits<float>::infinity(); 
    std::vector<float> u(n+1), v(m+1);
    std::vector<int> p(m+1), way(m+1); 

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<float> minv(m+1, INF);
        std::vector<char> used(m+1, false);

        do {
            used[j0] = true;
            int i0 = p[j0];
            float delta = INF;
            int j1 = 0;

            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    float cur = costMatrix[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }

        for (int j = 0; j <= m; ++j) {
            if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
            else minv[j] -= delta;
        }
        j0 = j1;
    } while (p[j0] != 0);

    do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
    } while (j0);
    }

    std::vector<int> result(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            result[p[j]-1] = j-1;
        }
    }
    return result;
}

std::vector<int> assignHungarianSlots(
    const std::vector<Vec3>& dronePositions,
    const std::vector<Vec3>& slotPositions
) {
    int n = (int)dronePositions.size();
    int m = (int)slotPositions.size();

    std::vector<std::vector<float>> costMatrix(n, std::vector<float>(m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            costMatrix[i][j] = squaredDist(dronePositions[i], slotPositions[j]);
        }
    }
    return hungarianMin(costMatrix);
}