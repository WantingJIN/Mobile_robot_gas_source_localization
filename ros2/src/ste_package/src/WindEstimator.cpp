#include <Eigen/SparseCholesky>
#include "ste_package/WindEstimator.h"
// Use shorter alias for base class access

// ============================================================
// Constructor
// ============================================================

GaussianWindEstimator::GaussianWindEstimator(
    float xmin, float xmax, float ymin, float ymax, int nx, int ny, float** obstacles, int obs_nbr, float sigma, float alpha, float c)
    : WindEstimator(xmin, xmax, ymin, ymax, nx, ny, obstacles, obs_nbr), sigma_(sigma), alpha_(alpha), c_(c)
{
    computeSDF();
    computeNormals();
}

// ============================================================
// Compute Signed Distance Field (walls + obstacles)
// ============================================================
void GaussianWindEstimator::computeSDF() {

    D_.resize(ny_, std::vector<float>(nx_, 1e9));
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            float x = X_[j][i], y = Y_[j][i];
            float d_wall = std::min(y, ymax_ - y);
            float d_val = d_wall;

            for (const auto& o : obstacles_) {
                float xmin = o[0], ymin = o[1], xmax = o[2], ymax = o[3];
                //printf("Obstacle: [%f, %f, %f, %f]\n", xmin, ymin, xmax, ymax);
                float dxo = std::max({xmin - x, (float)0.0, x - xmax});
                float dyo = std::max({ymin - y, (float)0.0, y - ymax});
                float unsigned_d = std::sqrt(dxo*dxo + dyo*dyo);
                bool inside = (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
                float signed_d = inside ? -unsigned_d : unsigned_d;
                if (std::abs(signed_d) < std::abs(d_val))
                    d_val = signed_d;
            }
            D_[j][i] = d_val;
        }
    }
}

// ============================================================
// Compute Normals from SDF
// ============================================================
void GaussianWindEstimator::computeNormals() {
    Nx_.assign(ny_, std::vector<float>(nx_, 0.0));
    Ny_.assign(ny_, std::vector<float>(nx_, 0.0));
    for (int j = 1; j < ny_ - 1; ++j) {
        for (int i = 1; i < nx_ - 1; ++i) {
            float dDx = (D_[j][i+1] - D_[j][i-1]) / (2*dx_);
            float dDy = (D_[j+1][i] - D_[j-1][i]) / (2*dy_);
            float mag = std::sqrt(dDx*dDx + dDy*dDy) + 1e-12;
            Nx_[j][i] = dDx / mag;
            Ny_[j][i] = dDy / mag;
        }
    }

    // for the boundaries, use forward/backward difference
    for (int j = 0; j < ny_; ++j) {
        // left boundary
        {
            int i = 0;
            float dDx = (D_[j][i+1] - D_[j][i]) / dx_;
            float dDy;
            if (j == 0)
                dDy = (D_[j+1][i] - D_[j][i]) / dy_;
            else if (j == ny_ - 1)
                dDy = (D_[j][i] - D_[j-1][i]) / dy_;
            else
                dDy = (D_[j+1][i] - D_[j-1][i]) / (2*dy_);
            float mag = std::sqrt(dDx*dDx + dDy*dDy) + 1e-12;
            Nx_[j][i] = dDx / mag;
            Ny_[j][i] = dDy / mag;
        }
        // right boundary
        {
            int i = nx_ - 1;
            float dDx = (D_[j][i] - D_[j][i-1]) / dx_;
            float dDy;
            if (j == 0)
                dDy = (D_[j+1][i] - D_[j][i]) / dy_;
            else if (j == ny_ - 1)
                dDy = (D_[j][i] - D_[j-1][i]) / dy_;
            else
                dDy = (D_[j+1][i] - D_[j-1][i]) / (2*dy_);
            float mag = std::sqrt(dDx*dDx + dDy*dDy) + 1e-12;
            Nx_[j][i] = dDx / mag;
            Ny_[j][i] = dDy / mag;
        }
    }
}

// ============================================================
// Estimate field (anisotropic Gaussian interpolation)
// ============================================================

// TODO - improvement: only proceed the new measured wind data and update the wind field estimation
void GaussianWindEstimator::estimateField(
        std::vector<float> meas_x, std::vector<float> meas_y, std::vector<double> u_meas, std::vector<double> v_meas)
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
        U_global_ = (1.0 - alpha_) * U_global_ + alpha_ * mean_U;
        V_global_ = (1.0 - alpha_) * V_global_ + alpha_ * mean_V;
    }   

    // ==========================================================
    // Step 2: Interpolation with updated background
    // ==========================================================
    U_field_.assign(ny_, std::vector<float>(nx_, 0.0));
    V_field_.assign(ny_, std::vector<float>(nx_, 0.0));
    W_field_.assign(ny_, std::vector<float>(nx_, 0.0));

    // convert measurement positions to grid indices

    for (int i = 0; i < u_meas.size(); ++i) {
        float U_i = u_meas[i], V_i = v_meas[i];
        float x_i = meas_x[i], y_i = meas_y[i];
        int idx_i, idy_i;
        world_pos_to_grid_indices(meas_x[i], meas_y[i], idx_i, idy_i);
    
        
        float d_local = std::abs(D_[idy_i][idx_i]);
        float n_x = Nx_[idy_i][idx_i], n_y = Ny_[idy_i][idx_i];
        float beta = 1.0 + c_ / (d_local + 1e-3);
        for (int jj = 0; jj < ny_; ++jj) {
            for (int ii = 0; ii < nx_; ++ii) {
                float dx_ = X_[jj][ii] - x_i;
                float dy_ = Y_[jj][ii] - y_i;
                float r_n = dx_ * n_x + dy_ * n_y;
                float r_t = -dx_ * n_y + dy_ * n_x;
                float w = std::exp(-(r_t*r_t + beta*r_n*r_n) / (2*sigma_*sigma_));
                //float w = std::exp(-(dx_*dx_ + dy_*dy_) / (2*sigma_*sigma_));
                U_field_[jj][ii] += w * U_i;
                V_field_[jj][ii] += w * V_i;
                W_field_[jj][ii] += w;
            }
        }   
    }

    for (int j = 0; j < ny_; ++j){
        for (int i = 0; i < nx_; ++i) {
            if (W_field_[j][i] > 1e-6) {
                U_field_[j][i] /= W_field_[j][i];
                V_field_[j][i] /= W_field_[j][i];
            } else {
                U_field_[j][i] = U_global_;
                V_field_[j][i] = V_global_;
            }
            
        }
    }
}

// ============================================================
// Divergence-free projection via Jacobi Poisson solver
// ============================================================
void GaussianWindEstimator::applyDivergenceFreeProjection(int max_iter, float tol) {

    std::vector<std::vector<float>> div(ny_, std::vector<float>(nx_, 0.0));
    for (int j = 1; j < ny_ - 1; ++j)
        for (int i = 1; i < nx_ - 1; ++i)
            div[j][i] = ((U_field_[j][i+1] - U_field_[j][i-1]) / (2*dx_) +
                         (V_field_[j+1][i] - V_field_[j-1][i]) / (2*dy_));

    std::vector<std::vector<float>> phi(ny_, std::vector<float>(nx_, 0.0));
    std::vector<std::vector<float>> phi_new = phi;

    for (int iter = 0; iter < max_iter; ++iter) {
        float max_res = 0;
        for (int j = 1; j < ny_ - 1; ++j)
            for (int i = 1; i < nx_ - 1; ++i) {
                float phi_e = phi[j][i+1], phi_w = phi[j][i-1];
                float phi_n = phi[j+1][i], phi_s = phi[j-1][i];
                phi_new[j][i] = 0.25 * (phi_e + phi_w + phi_n + phi_s -
                                        (dx_*dy_) * div[j][i]);
                max_res = std::max(max_res, std::abs(phi_new[j][i] - phi[j][i]));
            }
        phi.swap(phi_new);
        if (max_res < tol) break;
    }

    for (int j = 1; j < ny_  - 1; ++j)
        for (int i = 1; i < nx_ - 1; ++i) {
            float dphidx = (phi[j][i+1] - phi[j][i-1]) / (2*dx_);
            float dphidy = (phi[j+1][i] - phi[j-1][i]) / (2*dy_);
            U_field_[j][i] -= dphidx;
            V_field_[j][i] -= dphidy;
        }
}


/******************************Base flow + online correction*************************************/

BaseflowCorrectionWindEstimator::BaseflowCorrectionWindEstimator(
    float xmin, float xmax, float ymin, float ymax, int nx, int ny, float** obstacles, int obs_nbr, float RbfLengthScale, double lambda)
    : WindEstimator(xmin, xmax, ymin, ymax, nx, ny, obstacles, obs_nbr), RbfLengthScale_(RbfLengthScale), lambda_(lambda)
{
    N_ = nx_ * ny_;

    // Copy obstacles

    buildObstacleMask();
    solvePotentialFlow();
    computeVelocityFromPhi();

    // Build RBF basis (similar to Python: spacing ~ 1.0, length scale ~ 1.0)
    std::vector<Eigen::Vector2d> centers;
    //center locations, and x, y spacing; now the spacing are defined the same length scale
    buildRbfCenters(centers, RbfLengthScale_, RbfLengthScale_);
    buildRbfBasis(centers, RbfLengthScale_);

    // Initialize estimated field as base flow (can be overwritten in estimateField)
    u_est_ = u0_;
    v_est_ = v0_;
}

// ==================== helpers ====================

void BaseflowCorrectionWindEstimator::buildObstacleMask()
{
    obsMask_.setZero(ny_, nx_);

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            double xx = X_[j][i];
            double yy = Y_[j][i];
            bool inside = false;
            for (int k = 0; k < obstacles_.size(); ++k) {
                if (xx >= obstacles_[k][0] && xx <= obstacles_[k][2] &&
                    yy >= obstacles_[k][1] && yy <= obstacles_[k][3]) {
                    inside = true;
                    break;
                }
            }
            obsMask_(j, i) = inside ? 1 : 0;
        }
    }
}

void BaseflowCorrectionWindEstimator::solvePotentialFlow()
{
    using Triplet = Eigen::Triplet<double>;
    std::vector<Triplet> triplets;
    triplets.reserve(5 * N_);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(N_);

    auto isFluid = [&](int i, int j) -> bool {
        if (i < 0 || i >= nx_ || j < 0 || j >= ny_) return false;
        return obsMask_(j, i) == 0;
    };

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            int k = idx1D(i, j);

            // Dirichlet at left boundary (x = xmin) -> phi = 1
            if (i == 0) {
                triplets.emplace_back(k, k, 1.0);
                b(k) = 1.0;
                continue;
            }

            // Dirichlet at right boundary (x = xmax) -> phi = 0
            if (i == nx_ - 1) {
                triplets.emplace_back(k, k, 1.0);
                b(k) = 0.0;
                continue;
            }

            // Inside obstacle: pin to arbitrary constant
            if (obsMask_(j, i) == 1) {
                triplets.emplace_back(k, k, 1.0);
                b(k) = 0.5;
                continue;
            }

            double cCenter = 0.0;

            // X-direction
            int iL = i - 1;
            int iR = i + 1;
            bool leftFluid  = isFluid(iL, j);
            bool rightFluid = isFluid(iR, j);

            if (leftFluid && rightFluid) {
                triplets.emplace_back(k, idx1D(iL, j),  1.0 / (dx_ * dx_));
                triplets.emplace_back(k, idx1D(iR, j),  1.0 / (dx_ * dx_));
                cCenter -= 2.0 / (dx_ * dx_);
            } else if (leftFluid && !rightFluid) {
                triplets.emplace_back(k, idx1D(iL, j),  1.0 / (dx_ * dx_));
                cCenter -= 1.0 / (dx_ * dx_);
            } else if (!leftFluid && rightFluid) {
                triplets.emplace_back(k, idx1D(iR, j),  1.0 / (dx_ * dx_));
                cCenter -= 1.0 / (dx_ * dx_);
            }

            // Y-direction
            int jD = j - 1;
            int jU = j + 1;
            bool downFluid = isFluid(i, jD);
            bool upFluid   = isFluid(i, jU);

            if (downFluid && upFluid) {
                triplets.emplace_back(k, idx1D(i, jD),  1.0 / (dy_ * dy_));
                triplets.emplace_back(k, idx1D(i, jU),  1.0 / (dy_ * dy_));
                cCenter -= 2.0 / (dy_ * dy_);
            } else if (downFluid && !upFluid) {
                triplets.emplace_back(k, idx1D(i, jD),  1.0 / (dy_ * dy_));
                cCenter -= 1.0 / (dy_ * dy_);
            } else if (!downFluid && upFluid) {
                triplets.emplace_back(k, idx1D(i, jU),  1.0 / (dy_ * dy_));
                cCenter -= 1.0 / (dy_ * dy_);
            }

            triplets.emplace_back(k, k, cCenter);
        }
    }

    Eigen::SparseMatrix<double> A(N_, N_);
    A.setFromTriplets(triplets.begin(), triplets.end());

    std::cout << "Solving Laplace system of size " << A.rows() << " x " << A.cols() << std::endl;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    Eigen::VectorXd phiVec = solver.solve(b);

    phi_.resize(ny_, nx_);
    for (int j = 0; j < ny_; ++j)
        for (int i = 0; i < nx_; ++i)
            phi_(j, i) = phiVec(idx1D(i, j));
}

void BaseflowCorrectionWindEstimator::computeVelocityFromPhi()
{
    u0_.setZero(ny_, nx_);
    v0_.setZero(ny_, nx_);

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            if (obsMask_(j, i) == 1) {
                u0_(j, i) = 0.0;
                v0_(j, i) = 0.0;
                continue;
            }

            // u = d phi / dx
            if (i > 0 && i < nx_ - 1 &&
                obsMask_(j, i - 1) == 0 &&
                obsMask_(j, i + 1) == 0) {
                u0_(j, i) = (phi_(j, i + 1) - phi_(j, i - 1)) / (2.0 * dx_);
            } else if (i > 0 && obsMask_(j, i - 1) == 0) {
                u0_(j, i) = (phi_(j, i) - phi_(j, i - 1)) / dx_;
            } else if (i < nx_ - 1 && obsMask_(j, i + 1) == 0) {
                u0_(j, i) = (phi_(j, i + 1) - phi_(j, i)) / dx_;
            } else {
                u0_(j, i) = 0.0;
            }

            // v = d phi / dy
            if (j > 0 && j < ny_ - 1 &&
                obsMask_(j - 1, i) == 0 &&
                obsMask_(j + 1, i) == 0) {
                v0_(j, i) = (phi_(j + 1, i) - phi_(j - 1, i)) / (2.0 * dy_);
            } else if (j > 0 && obsMask_(j - 1, i) == 0) {
                v0_(j, i) = (phi_(j, i) - phi_(j - 1, i)) / dy_;
            } else if (j < ny_ - 1 && obsMask_(j + 1, i) == 0) {
                v0_(j, i) = (phi_(j + 1, i) - phi_(j, i)) / dy_;
            } else {
                v0_(j, i) = 0.0;
            }
        }
    }
}

void BaseflowCorrectionWindEstimator::buildRbfCenters(
    std::vector<Eigen::Vector2d> &centers,
    double spacingX, double spacingY)
{
    // Avoid boundaries like Python version
    for (double cx = xmin_ + spacingX * 0.5; cx < xmax_; cx += spacingX) {
        for (double cy = ymin_ + spacingY * 0.5; cy < ymax_; cy += spacingY) {
            centers.emplace_back(cx, cy);
        }
    }
}

void BaseflowCorrectionWindEstimator::buildRbfBasis(
    const std::vector<Eigen::Vector2d> &centers,
    double lengthScale)
{
    const int M = static_cast<int>(centers.size());
    rbfU_.resize(M);
    rbfV_.resize(M);

    double l2 = lengthScale * lengthScale;

    for (int m = 0; m < M; ++m) {
        rbfU_[m].setZero(ny_, nx_);
        rbfV_[m].setZero(ny_, nx_);

        double cx = centers[m](0);
        double cy = centers[m](1);

        for (int j = 0; j < ny_; ++j) {
            double yy = Y_[j][0];
            for (int i = 0; i < nx_; ++i) {
                double xx = X_[j][i];
                double dx = xx - cx;
                double dy = yy - cy;
                double r2 = dx * dx + dy * dy;
                double xi = std::exp(-r2 / (2.0 * l2));

                double u = -(yy - cy) / l2 * xi;  // d xi / dy
                double v =  (xx - cx) / l2 * xi;  // - d xi / dx

                rbfU_[m](j, i) = u;
                rbfV_[m](j, i) = v;
            }
        }
    }
}

// Build H, z, w (grid-based, no interpolation: idx/idy directly index cells)
void BaseflowCorrectionWindEstimator::buildDesignMatrix(
    const std::vector<int> &idx, const std::vector<int> &idy,
    const std::vector<double> &u_meas,
    const std::vector<double> &v_meas,
    Eigen::MatrixXd &H,
    Eigen::VectorXd &z,
    Eigen::VectorXd &w) const
{
    const int K = static_cast<int>(u_meas.size());
    const int M = static_cast<int>(rbfU_.size());

    H.resize(2 * K, 1 + M);
    z.resize(2 * K);
    w.resize(2 * K);

    H.setZero();
    z.setZero();
    w.setOnes();      // you can change weights downstream if you want

    //FIXME - be careful with idx, maybe it is not the grid index but the measurement index
    for (int k = 0; k < K; ++k) {
        int i = idx[k];
        int j = idy[k];

        // base flow at this grid cell
        double u0k = u0_(j, i);
        double v0k = v0_(j, i);

        // u-row
        int row_u = 2 * k;
        H(row_u, 0) = u0k;
        for (int m = 0; m < M; ++m)
            H(row_u, 1 + m) = rbfU_[m](j, i);
        z(row_u) = u_meas[k];

        // v-row
        int row_v = 2 * k + 1;
        H(row_v, 0) = v0k;
        for (int m = 0; m < M; ++m)
            H(row_v, 1 + m) = rbfV_[m](j, i);
        z(row_v) = v_meas[k];
        
        // TODO: hard-coded weights with respect to x axis of the sampling location
        // Example of down-weighting downstream region (like x > 6.0)
        double weight = 1.0;
        if (X_[j][i] < 10.0) {
            weight = 0.5;
        }
        w(row_u) = weight;
        w(row_v) = weight;
    }
}

Eigen::VectorXd BaseflowCorrectionWindEstimator::solveRegularizedLS(
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &z,
    const Eigen::VectorXd &w,
    double lambda) const
{
    const int p = H.cols();
    Eigen::VectorXd sqrtw = w.array().sqrt();
    Eigen::MatrixXd Hw = H;
    Eigen::VectorXd zw = z;

    for (int i = 0; i < H.rows(); ++i) {
        Hw.row(i) *= sqrtw(i);
        zw(i) *= sqrtw(i);
    }

    // normal equations: (H^T H + lam*Reg) theta = H^T z
    Eigen::MatrixXd Reg = Eigen::MatrixXd::Identity(p, p);
    Reg(0, 0) = 0.0;   // do not regularize S

    Eigen::MatrixXd A = Hw.transpose() * Hw +lambda * Reg;
    Eigen::VectorXd b = Hw.transpose() * zw;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
    Eigen::VectorXd theta = ldlt.solve(b);
    return theta;
}

// ==================== public API ====================

void BaseflowCorrectionWindEstimator::estimateField(
    std::vector<float> meas_x, std::vector<float> meas_y,
    std::vector<double> u_meas,
    std::vector<double> v_meas)
{
    U_field_.assign(ny_, std::vector<float>(nx_, 0.0));
    V_field_.assign(ny_, std::vector<float>(nx_, 0.0));
    W_field_.assign(ny_, std::vector<float>(nx_, 0.0));
    
    if (u_meas.size() != v_meas.size()) {
        std::cerr << "estimateField: u_meas and v_meas size mismatch" << std::endl;
        return;
    }

    // 1) Build design matrix
    // convert measurement positions to grid indices 
    std::vector<int> idx(u_meas.size()), idy(u_meas.size());
    for (int i = 0; i < u_meas.size(); ++i) {
        world_pos_to_grid_indices(meas_x[i], meas_y[i], idx[i], idy[i]);
    }

    Eigen::MatrixXd H;
    Eigen::VectorXd z, w;
    buildDesignMatrix(idx, idy, u_meas, v_meas, H, z, w);

    // 2) Solve for theta = [S, a_1, ..., a_M]^T
    double lambda = lambda_* exp(-0.01 * u_meas.size()); // adaptive regularization based on number of measurements
    Eigen::VectorXd theta = solveRegularizedLS(H, z, w, lambda);

    double S_hat = theta(0);
    int    M     = static_cast<int>(rbfU_.size());

    std::cout << "Estimated S_hat = " << S_hat << std::endl;
    std::cout << "Estimated a coefficients: ";

    // 3) Rebuild field on full grid
    u_est_ = S_hat * u0_;
    v_est_ = S_hat * v0_;

    for (int m = 0; m < M; ++m) {
        double a = theta(1 + m);
        u_est_.noalias() += a * rbfU_[m];
        v_est_.noalias() += a * rbfV_[m];
        std::cout << a << " ";
    }
    std::cout << std::endl;

    // Zero inside obstacles
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i){
            if (obsMask_(j, i) == 1) {
                u_est_(j, i) = 0.0;
                v_est_(j, i) = 0.0;
            }
        }
    }
        
    // copy the value to base class fields U_field_, V_field_
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            U_field_[j][i] = static_cast<float>(u_est_(j, i));
            V_field_[j][i] = static_cast<float>(v_est_(j, i));
        }
    }
}