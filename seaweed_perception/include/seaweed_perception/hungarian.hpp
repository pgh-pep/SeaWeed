/**
 * Hungarian algorithm (solving assignment problem)
 *
 * Based on implementation from cp-algorithms.com
 * Original: https://cp-algorithms.com/graph/hungarian-algorithm.html
 */

#pragma once

#include <algorithm>
#include <vector>

struct HungarianResult
{
    std::vector<int> row_to_col; // row_to_col[i] = assigned column for row i, -1 if unmatched
    std::vector<int> col_to_row; // col_to_row[j] = assigned row for column j, -1 if unmatched
};

inline HungarianResult hungarian_solve(const std::vector<std::vector<float>> &cost, float max_cost = 1e9f)
{
    if (cost.empty() || cost[0].empty())
    {
        return {{}, {}};
    }

    int rows = static_cast<int>(cost.size());
    int cols = static_cast<int>(cost[0].size());
    int n = std::max(rows, cols);

    constexpr float INF = 1e9f;

    // fill square matrix with inf to pad
    std::vector<std::vector<float>> c(n, std::vector<float>(n, INF));
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            c[i][j] = cost[i][j];
        }
    }

    std::vector<float> u(n + 1, 0.0f);
    std::vector<float> v(n + 1, 0.0f);

    // p[j] = row assigned to column j (1-indexed, 0 = unassigned)
    std::vector<int> p(n + 1, 0);

    // way[j] = previous column
    std::vector<int> way(n + 1, 0);

    for (int i = 1; i <= n; ++i)
    {
        p[0] = i;
        int j0 = 0;

        std::vector<float> minv(n + 1, INF);
        std::vector<bool> used(n + 1, false);

        do
        {
            used[j0] = true;
            int i0 = p[j0];
            int j1 = 0;
            float delta = INF;

            for (int j = 1; j <= n; ++j)
            {
                if (!used[j])
                {
                    float cur = c[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j])
                    {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta)
                    {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= n; ++j)
            {
                if (used[j])
                {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else
                {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] != 0);

        do
        {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    HungarianResult result;
    result.row_to_col.assign(rows, -1);
    result.col_to_row.assign(cols, -1);

    for (int j = 1; j <= n; ++j)
    {
        int i = p[j] - 1;
        int col = j - 1;

        if (i >= 0 && i < rows && col < cols)
        {
            if (cost[i][col] <= max_cost)
            {
                result.row_to_col[i] = col;
                result.col_to_row[col] = i;
            }
        }
    }

    return result;
}