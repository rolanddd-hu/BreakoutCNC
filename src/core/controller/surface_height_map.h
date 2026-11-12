#pragma once
// Probed surface height map (bilinear / bicubic warp of the toolpath).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rnr::core {

// ============================================================================
// Probed Workpiece Surface Height Map (Bilinear & Bicubic Spline Warping)
// ============================================================================
struct SurfaceHeightMap {
    bool                is_valid = false;
    double              min_x    = 0.0;
    double              max_x    = 0.0;
    double              min_y    = 0.0;
    double              max_y    = 0.0;
    int                 points_x = 0;
    int                 points_y = 0;
    std::vector<double> grid_z{}; // Row-major: row j (Y), col i (X) -> j * points_x + i
    double              smoothness = 0.5; // 0.0 = Pure Bilinear (Rigid), 1.0 = Pure Bicubic Spline (Smooth)

    // Statistics
    double min_z          = 0.0;
    double max_z          = 0.0;
    double mean_z         = 0.0;
    double peak_to_peak_z = 0.0;

    void clear() {
        is_valid = false;
        min_x = max_x = min_y = max_y = 0.0;
        points_x = points_y = 0;
        grid_z.clear();
        smoothness = 0.5;
        min_z = max_z = mean_z = peak_to_peak_z = 0.0;
    }

    void recalculateStats() {
        if (grid_z.empty()) {
            min_z = max_z = mean_z = peak_to_peak_z = 0.0;
            return;
        }
        min_z = grid_z.front();
        max_z = grid_z.front();
        double sum = 0.0;
        for (double z : grid_z) {
            if (z < min_z) min_z = z;
            if (z > max_z) max_z = z;
            sum += z;
        }
        mean_z = sum / static_cast<double>(grid_z.size());
        peak_to_peak_z = max_z - min_z;
    }

    double getInterpolatedZ(double x, double y) const {
        return getInterpolatedZWithSmoothness(x, y, smoothness);
    }

    double getInterpolatedZWithSmoothness(double x, double y, double user_smoothness) const {
        if (!is_valid || points_x < 2 || points_y < 2 ||
            grid_z.size() != static_cast<size_t>(points_x * points_y))
        {
            return 0.0;
        }
        double span_x = max_x - min_x;
        double span_y = max_y - min_y;
        if (span_x <= 1e-6 || span_y <= 1e-6) return 0.0;

        // Clamp x, y to bounding box for smooth edge extrapolation
        double cx = std::clamp(x, min_x, max_x);
        double cy = std::clamp(y, min_y, max_y);

        double u = ((cx - min_x) / span_x) * static_cast<double>(points_x - 1);
        double v = ((cy - min_y) / span_y) * static_cast<double>(points_y - 1);

        int i = std::clamp(static_cast<int>(std::floor(u)), 0, points_x - 2);
        int j = std::clamp(static_cast<int>(std::floor(v)), 0, points_y - 2);

        double t = u - static_cast<double>(i); // [0.0, 1.0]
        double s = v - static_cast<double>(j); // [0.0, 1.0]

        double z00 = grid_z[static_cast<size_t>(j * points_x + i)];
        double z10 = grid_z[static_cast<size_t>(j * points_x + (i + 1))];
        double z01 = grid_z[static_cast<size_t>((j + 1) * points_x + i)];
        double z11 = grid_z[static_cast<size_t>((j + 1) * points_x + (i + 1))];

        double z_bilinear = (1.0 - t) * (1.0 - s) * z00 +
                            t * (1.0 - s) * z10 +
                            (1.0 - t) * s * z01 +
                            t * s * z11;

        double alpha = std::clamp(user_smoothness, 0.0, 1.0);
        if (alpha <= 1e-4 || points_x < 3 || points_y < 3) {
            return z_bilinear;
        }

        // Bicubic Catmull-Rom Spline Interpolation:
        // C1 continuous tangents matching adjacent cell boundaries
        auto sample_node = [&](int col, int row) -> double {
            col = std::clamp(col, 0, points_x - 1);
            row = std::clamp(row, 0, points_y - 1);
            return grid_z[static_cast<size_t>(row * points_x + col)];
        };

        // 1D Catmull-Rom polynomial evaluator across p_{-1}, p_0, p_1, p_2
        auto catmull_rom_1d = [](double p_minus, double p0, double p1, double p_plus, double frac) -> double {
            double a = -0.5 * p_minus + 1.5 * p0 - 1.5 * p1 + 0.5 * p_plus;
            double b = p_minus - 2.5 * p0 + 2.0 * p1 - 0.5 * p_plus;
            double c = -0.5 * p_minus + 0.5 * p1;
            double d = p0;
            return ((a * frac + b) * frac + c) * frac + d;
        };

        // Evaluate 4 rows in X at parameter t
        double row_vals[4];
        for (int r = 0; r < 4; ++r) {
            int row_idx = j - 1 + r;
            double p_m = sample_node(i - 1, row_idx);
            double p_0 = sample_node(i,     row_idx);
            double p_1 = sample_node(i + 1, row_idx);
            double p_p = sample_node(i + 2, row_idx);
            row_vals[r] = catmull_rom_1d(p_m, p_0, p_1, p_p, t);
        }

        // Evaluate across the 4 row values in Y at parameter s
        double z_bicubic = catmull_rom_1d(row_vals[0], row_vals[1], row_vals[2], row_vals[3], s);

        // Blend bilinear stiffness and bicubic curvature based on smoothness alpha
        return (1.0 - alpha) * z_bilinear + alpha * z_bicubic;
    }
};

} // namespace rnr::core
