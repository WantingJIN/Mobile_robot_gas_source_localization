#include "WindEstimator.h"

// Use shorter alias for base class access

// ============================================================
// Constructor
// ============================================================

GaussianWindEstimator::GaussianWindEstimator(
    float xmin, float xmax, float ymin, float ymax, int nx, int ny, float** obstacles, int obs_nbr)
    : WindEstimator(xmin, xmax, ymin, ymax, nx, ny, obstacles, obs_nbr)
{
    computeSDF();
    computeNormals();
}

// ============================================================
// Compute Signed Distance Field (walls + obstacles)
// ============================================================
void GaussianWindEstimator::computeSDF() {

    D.resize(ny, std::vector<float>(nx, 1e9));

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            float x = X[j][i], y = Y[j][i];
            float d_wall = std::min(y, ymax - y);
            float d_val = d_wall;

            for (const auto& o : obstacles) {
                float xmin = o[0], ymin = o[1], xmax = o[2], ymax = o[3];
                float dxo = std::max({xmin - x, (float)0.0, x - xmax});
                float dyo = std::max({ymin - y, (float)0.0, y - ymax});
                float unsigned_d = std::sqrt(dxo*dxo + dyo*dyo);
                bool inside = (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
                float signed_d = inside ? -unsigned_d : unsigned_d;
                if (std::abs(signed_d) < std::abs(d_val))
                    d_val = signed_d;
            }
            D[j][i] = d_val;
        }
    }
}

// ============================================================
// Compute Normals from SDF
// ============================================================
void GaussianWindEstimator::computeNormals() {
    Nx.assign(ny, std::vector<float>(nx, 0.0));
    Ny.assign(ny, std::vector<float>(nx, 0.0));
    for (int j = 1; j < ny - 1; ++j)
        for (int i = 1; i < nx - 1; ++i) {
            float dDx = (D[j][i+1] - D[j][i-1]) / (2*dx);
            float dDy = (D[j+1][i] - D[j-1][i]) / (2*dy);
            float mag = std::sqrt(dDx*dDx + dDy*dDy) + 1e-12;
            Nx[j][i] = dDx / mag;
            Ny[j][i] = dDy / mag;
        }
}

// ============================================================
// Estimate field (anisotropic Gaussian interpolation)
// ============================================================

// FIXME - the recalculation of global wind flow is not here
// TODO - improvement: only proceed the new measured wind data and update the wind field estimation
void GaussianWindEstimator::estimateField(
        int idx[], int idy[], std::vector<double> u_meas, std::vector<double> v_meas,
        float sigma, float alpha, float c)
{
    // ==========================================================
    // Step 1: Update background (global average) wind
    // ==========================================================
    float mean_U = 0.0, mean_V = 0.0;
    if (!u_meas.empty()) {
        for (int i = 0; i < u_meas.size(); ++i) {
            mean_U += u_meas[i];
            mean_V += v_meas[i];
        }
        mean_U /= u_meas.size();
        mean_V /= v_meas.size();

        // Exponential moving average
        U_global_ = (1.0 - alpha) * U_global_ + alpha * mean_U;
        V_global_ = (1.0 - alpha) * V_global_ + alpha * mean_V;
    }

    // ==========================================================
    // Step 2: Interpolation with updated background
    // ==========================================================
    U_field.assign(ny, std::vector<float>(nx, 0.0));
    V_field.assign(ny, std::vector<float>(nx, 0.0));
    W_field.assign(ny, std::vector<float>(nx, 0.0));

    for (int i = 0; i < u_meas.size(); ++i) {
        float idx_i = idx[i], idy_i = idy[i], U_i = u_meas[i], V_i = v_meas[i];
        // int j = std::clamp(static_cast<int>(std::round(y_i / dy)), 0, ny - 1);
        // int i = std::clamp(static_cast<int>(std::round(x_i / dx)), 0, nx - 1);
        float x_i, y_i;
        indices_to_position_2D(idx_i, idy_i, &x_i, &y_i);
        
        // FIXME -  check here; why j and i are swapped?
        float d_local = std::abs(D[idy_i][idx_i]);
        float n_x = Nx[idy_i][idx_i], n_y = Ny[idy_i][idx_i];
        float beta = 1.0 + c / (d_local + 1e-3);

        for (int jj = 0; jj < ny; ++jj)
            for (int ii = 0; ii < nx; ++ii) {
                float dx_ = X[jj][ii] - x_i;
                float dy_ = Y[jj][ii] - y_i;
                float r_n = dx_ * n_x + dy_ * n_y;
                float r_t = -dx_ * n_y + dy_ * n_x;
                float w = std::exp(-(r_t*r_t + beta*r_n*r_n) / (2*sigma*sigma));
                U_field[jj][ii] += w * U_i;
                V_field[jj][ii] += w * V_i;
                W_field[jj][ii] += w;
            }
    }

    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            if (W_field[j][i] > 1e-6) {
                U_field[j][i] /= W_field[j][i];
                V_field[j][i] /= W_field[j][i];
            } else {
                U_field[j][i] = U_global_;
                V_field[j][i] = V_global_;
            }
}

// ============================================================
// Divergence-free projection via Jacobi Poisson solver
// ============================================================
void GaussianWindEstimator::applyDivergenceFreeProjection(int max_iter, float tol) {

    std::vector<std::vector<float>> div(ny, std::vector<float>(nx, 0.0));
    for (int j = 1; j < ny - 1; ++j)
        for (int i = 1; i < nx - 1; ++i)
            div[j][i] = ((U_field[j][i+1] - U_field[j][i-1]) / (2*dx) +
                         (V_field[j+1][i] - V_field[j-1][i]) / (2*dy));

    std::vector<std::vector<float>> phi(ny, std::vector<float>(nx, 0.0));
    std::vector<std::vector<float>> phi_new = phi;

    for (int iter = 0; iter < max_iter; ++iter) {
        float max_res = 0;
        for (int j = 1; j < ny - 1; ++j)
            for (int i = 1; i < nx - 1; ++i) {
                float phi_e = phi[j][i+1], phi_w = phi[j][i-1];
                float phi_n = phi[j+1][i], phi_s = phi[j-1][i];
                phi_new[j][i] = 0.25 * (phi_e + phi_w + phi_n + phi_s -
                                        (dx*dy) * div[j][i]);
                max_res = std::max(max_res, std::abs(phi_new[j][i] - phi[j][i]));
            }
        phi.swap(phi_new);
        if (max_res < tol) break;
    }

    for (int j = 1; j < ny - 1; ++j)
        for (int i = 1; i < nx - 1; ++i) {
            float dphidx = (phi[j][i+1] - phi[j][i-1]) / (2*dx);
            float dphidy = (phi[j+1][i] - phi[j-1][i]) / (2*dy);
            U_field[j][i] -= dphidx;
            V_field[j][i] -= dphidy;
        }
}
