#include "WindEstimator.h"

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <cmath>
#include <iostream>

WindEstimator::WindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                             float **obstacle_list, int obs_nbr)
    : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), nx_(nx), ny_(ny)
{
    for (int i = 0; i < obs_nbr; ++i) {
        std::array<float, 4> obs = {
            obstacle_list[i][0], obstacle_list[i][1],
            obstacle_list[i][2], obstacle_list[i][3]
        };
        obstacles_.push_back(obs);
    }

    dx_ = (xmax_ - xmin_) / static_cast<float>(nx_ - 1);
    dy_ = (ymax_ - ymin_) / static_cast<float>(ny_ - 1);
    buildGrid();
}

void WindEstimator::buildGrid()
{
    X_.assign(ny_, std::vector<float>(nx_, 0.0f));
    Y_.assign(ny_, std::vector<float>(nx_, 0.0f));
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            X_[j][i] = xmin_ + i * dx_;
            Y_[j][i] = ymin_ + j * dy_;
        }
    }
}

void WindEstimator::world_pos_to_grid_indices(float x, float y, int &i, int &j) const
{
    i = clamp_to_grid_x(world_to_i(x));
    j = clamp_to_grid_y(world_to_j(y));
}

int WindEstimator::clamp_to_grid_x(int i) const
{
    return std::max(0, std::min(i, nx_ - 1));
}

int WindEstimator::clamp_to_grid_y(int j) const
{
    return std::max(0, std::min(j, ny_ - 1));
}

int WindEstimator::world_to_i(float x) const
{
    return static_cast<int>(std::floor((x - xmin_) / dx_));
}

int WindEstimator::world_to_j(float y) const
{
    return static_cast<int>(std::floor((y - ymin_) / dy_));
}

GaussianWindEstimator::GaussianWindEstimator(float xmin, float xmax, float ymin, float ymax,
                                             int nx, int ny, float **obstacles, int obs_nbr,
                                             float sigma, float alpha, float c)
    : WindEstimator(xmin, xmax, ymin, ymax, nx, ny, obstacles, obs_nbr),
      sigma_(sigma), alpha_(alpha), c_(c), U_global_(0.0f), V_global_(0.0f)
{
    computeSDF();
    computeNormals();
}

void GaussianWindEstimator::computeSDF()
{
    D_.assign(ny_, std::vector<float>(nx_, 1e9f));
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            float x = X_[j][i];
            float y = Y_[j][i];
            float d_wall = std::min(y - ymin_, ymax_ - y);
            float d_val = d_wall;

            for (size_t k = 0; k < obstacles_.size(); ++k) {
                const std::array<float, 4> &o = obstacles_[k];
                float dxo = std::max(std::max(o[0] - x, 0.0f), x - o[2]);
                float dyo = std::max(std::max(o[1] - y, 0.0f), y - o[3]);
                float unsigned_d = std::sqrt(dxo * dxo + dyo * dyo);
                bool inside = (x >= o[0] && x <= o[2] && y >= o[1] && y <= o[3]);
                float signed_d = inside ? -unsigned_d : unsigned_d;
                if (std::fabs(signed_d) < std::fabs(d_val))
                    d_val = signed_d;
            }
            D_[j][i] = d_val;
        }
    }
}

void GaussianWindEstimator::computeNormals()
{
    Nx_.assign(ny_, std::vector<float>(nx_, 0.0f));
    Ny_.assign(ny_, std::vector<float>(nx_, 0.0f));

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            int il = std::max(0, i - 1);
            int ir = std::min(nx_ - 1, i + 1);
            int jb = std::max(0, j - 1);
            int jt = std::min(ny_ - 1, j + 1);
            float ddx = (D_[j][ir] - D_[j][il]) / (std::max(1, ir - il) * dx_);
            float ddy = (D_[jt][i] - D_[jb][i]) / (std::max(1, jt - jb) * dy_);
            float mag = std::sqrt(ddx * ddx + ddy * ddy) + 1e-12f;
            Nx_[j][i] = ddx / mag;
            Ny_[j][i] = ddy / mag;
        }
    }
}

void GaussianWindEstimator::estimateField(const std::vector<float> &meas_x,
                                          const std::vector<float> &meas_y,
                                          const std::vector<float> &u_meas,
                                          const std::vector<float> &v_meas)
{
    if (u_meas.empty() || u_meas.size() != v_meas.size() || u_meas.size() != meas_x.size())
        return;

    float mean_u = 0.0f;
    float mean_v = 0.0f;
    for (size_t i = 0; i < u_meas.size(); ++i) {
        mean_u += u_meas[i];
        mean_v += v_meas[i];
    }
    mean_u /= static_cast<float>(u_meas.size());
    mean_v /= static_cast<float>(v_meas.size());
    U_global_ = (1.0f - alpha_) * U_global_ + alpha_ * mean_u;
    V_global_ = (1.0f - alpha_) * V_global_ + alpha_ * mean_v;

    U_field_.assign(ny_, std::vector<float>(nx_, 0.0f));
    V_field_.assign(ny_, std::vector<float>(nx_, 0.0f));
    W_field_.assign(ny_, std::vector<float>(nx_, 0.0f));

    for (size_t k = 0; k < u_meas.size(); ++k) {
        int idx_i = 0;
        int idy_i = 0;
        world_pos_to_grid_indices(meas_x[k], meas_y[k], idx_i, idy_i);

        float d_local = std::fabs(D_[idy_i][idx_i]);
        float n_x = Nx_[idy_i][idx_i];
        float n_y = Ny_[idy_i][idx_i];
        float beta = 1.0f + c_ / (d_local + 1e-3f);

        for (int j = 0; j < ny_; ++j) {
            for (int i = 0; i < nx_; ++i) {
                float dx = X_[j][i] - meas_x[k];
                float dy = Y_[j][i] - meas_y[k];
                float r_n = dx * n_x + dy * n_y;
                float r_t = -dx * n_y + dy * n_x;
                float weight = std::exp(-(r_t * r_t + beta * r_n * r_n) / (2.0f * sigma_ * sigma_));
                U_field_[j][i] += weight * u_meas[k];
                V_field_[j][i] += weight * v_meas[k];
                W_field_[j][i] += weight;
            }
        }
    }

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            if (W_field_[j][i] > 1e-6f) {
                U_field_[j][i] /= W_field_[j][i];
                V_field_[j][i] /= W_field_[j][i];
            } else {
                U_field_[j][i] = U_global_;
                V_field_[j][i] = V_global_;
            }
        }
    }
}

BaseflowCorrectionWindEstimator::BaseflowCorrectionWindEstimator(
    float xmin, float xmax, float ymin, float ymax, int nx, int ny,
    float **obstacles, int obs_nbr, float rbf_length_scale, float lambda)
    : WindEstimator(xmin, xmax, ymin, ymax, nx, ny, obstacles, obs_nbr),
      rbf_length_scale_(rbf_length_scale), lambda_(lambda)
{
    N_ = nx_ * ny_;
    buildObstacleMask();
    solvePotentialFlow();
    computeVelocityFromPhi();

    std::vector<Eigen::Vector2f> centers;
    buildRbfCenters(centers, rbf_length_scale_, rbf_length_scale_);
    buildRbfBasis(centers, rbf_length_scale_);

    u_est_ = u0_;
    v_est_ = v0_;
}

void BaseflowCorrectionWindEstimator::buildObstacleMask()
{
    obsMask_.setZero(ny_, nx_);
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            float x = X_[j][i];
            float y = Y_[j][i];
            bool inside = false;
            for (size_t k = 0; k < obstacles_.size(); ++k) {
                const std::array<float, 4> &o = obstacles_[k];
                if (x >= o[0] && x <= o[2] && y >= o[1] && y <= o[3]) {
                    inside = true;
                    break;
                }
            }
            obsMask_(j, i) = inside ? 1.0f : 0.0f;
        }
    }
}

void BaseflowCorrectionWindEstimator::solvePotentialFlow()
{
    typedef Eigen::Triplet<float> Triplet;
    std::vector<Triplet> triplets;
    triplets.reserve(5 * N_);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(N_);

    auto is_fluid = [&](int i, int j) -> bool {
        if (i < 0 || i >= nx_ || j < 0 || j >= ny_)
            return false;
        return obsMask_(j, i) == 0.0f;
    };

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            int k = idx1D(i, j);
            if (i == 0) {
                triplets.push_back(Triplet(k, k, 1.0f));
                b(k) = 1.0f;
                continue;
            }
            if (i == nx_ - 1) {
                triplets.push_back(Triplet(k, k, 1.0f));
                b(k) = 0.0f;
                continue;
            }
            if (obsMask_(j, i) == 1.0f) {
                triplets.push_back(Triplet(k, k, 1.0f));
                b(k) = 0.5f;
                continue;
            }

            float center = 0.0f;
            if (is_fluid(i - 1, j)) {
                triplets.push_back(Triplet(k, idx1D(i - 1, j), 1.0f / (dx_ * dx_)));
                center -= 1.0f / (dx_ * dx_);
            }
            if (is_fluid(i + 1, j)) {
                triplets.push_back(Triplet(k, idx1D(i + 1, j), 1.0f / (dx_ * dx_)));
                center -= 1.0f / (dx_ * dx_);
            }
            if (is_fluid(i, j - 1)) {
                triplets.push_back(Triplet(k, idx1D(i, j - 1), 1.0f / (dy_ * dy_)));
                center -= 1.0f / (dy_ * dy_);
            }
            if (is_fluid(i, j + 1)) {
                triplets.push_back(Triplet(k, idx1D(i, j + 1), 1.0f / (dy_ * dy_)));
                center -= 1.0f / (dy_ * dy_);
            }
            triplets.push_back(Triplet(k, k, center));
        }
    }

    Eigen::SparseMatrix<float> A(N_, N_);
    A.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Baseflow wind estimator: potential-flow factorization failed" << std::endl;
        phi_.setZero(ny_, nx_);
        return;
    }
    Eigen::VectorXf phi_vec = solver.solve(b);
    if (solver.info() != Eigen::Success) {
        std::cerr << "Baseflow wind estimator: potential-flow solve failed" << std::endl;
        phi_.setZero(ny_, nx_);
        return;
    }

    phi_.resize(ny_, nx_);
    for (int j = 0; j < ny_; ++j)
        for (int i = 0; i < nx_; ++i)
            phi_(j, i) = phi_vec(idx1D(i, j));
}

void BaseflowCorrectionWindEstimator::computeVelocityFromPhi()
{
    u0_.setZero(ny_, nx_);
    v0_.setZero(ny_, nx_);

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            if (obsMask_(j, i) == 1.0f)
                continue;

            if (i > 0 && i < nx_ - 1 && obsMask_(j, i - 1) == 0.0f && obsMask_(j, i + 1) == 0.0f)
                u0_(j, i) = (phi_(j, i + 1) - phi_(j, i - 1)) / (2.0f * dx_);
            else if (i > 0 && obsMask_(j, i - 1) == 0.0f)
                u0_(j, i) = (phi_(j, i) - phi_(j, i - 1)) / dx_;
            else if (i < nx_ - 1 && obsMask_(j, i + 1) == 0.0f)
                u0_(j, i) = (phi_(j, i + 1) - phi_(j, i)) / dx_;

            if (j > 0 && j < ny_ - 1 && obsMask_(j - 1, i) == 0.0f && obsMask_(j + 1, i) == 0.0f)
                v0_(j, i) = (phi_(j + 1, i) - phi_(j - 1, i)) / (2.0f * dy_);
            else if (j > 0 && obsMask_(j - 1, i) == 0.0f)
                v0_(j, i) = (phi_(j, i) - phi_(j - 1, i)) / dy_;
            else if (j < ny_ - 1 && obsMask_(j + 1, i) == 0.0f)
                v0_(j, i) = (phi_(j + 1, i) - phi_(j, i)) / dy_;
        }
    }
}

void BaseflowCorrectionWindEstimator::buildRbfCenters(std::vector<Eigen::Vector2f> &centers,
                                                      float spacing_x,
                                                      float spacing_y)
{
    for (float cx = xmin_ + spacing_x * 0.5f; cx < xmax_; cx += spacing_x)
        for (float cy = ymin_ + spacing_y * 0.5f; cy < ymax_; cy += spacing_y)
            centers.push_back(Eigen::Vector2f(cx, cy));
}

void BaseflowCorrectionWindEstimator::buildRbfBasis(const std::vector<Eigen::Vector2f> &centers,
                                                    float length_scale)
{
    int M = static_cast<int>(centers.size());
    rbfU_.resize(M);
    rbfV_.resize(M);
    float l2 = length_scale * length_scale;

    for (int m = 0; m < M; ++m) {
        rbfU_[m].setZero(ny_, nx_);
        rbfV_[m].setZero(ny_, nx_);
        float cx = centers[m](0);
        float cy = centers[m](1);
        for (int j = 0; j < ny_; ++j) {
            float y = Y_[j][0];
            for (int i = 0; i < nx_; ++i) {
                float x = X_[j][i];
                float dx = x - cx;
                float dy = y - cy;
                float xi = std::exp(-(dx * dx + dy * dy) / (2.0f * l2));
                rbfU_[m](j, i) = -(y - cy) / l2 * xi;
                rbfV_[m](j, i) = (x - cx) / l2 * xi;
            }
        }
    }
}

void BaseflowCorrectionWindEstimator::buildDesignMatrix(const std::vector<int> &idx,
                                                        const std::vector<int> &idy,
                                                        const std::vector<float> &u_meas,
                                                        const std::vector<float> &v_meas,
                                                        Eigen::MatrixXf &H,
                                                        Eigen::VectorXf &z,
                                                        Eigen::VectorXf &w) const
{
    int K = static_cast<int>(u_meas.size());
    int M = static_cast<int>(rbfU_.size());

    H.setZero(2 * K, 1 + M);
    z.setZero(2 * K);
    w.setOnes(2 * K);

    for (int k = 0; k < K; ++k) {
        int i = idx[k];
        int j = idy[k];
        int row_u = 2 * k;
        int row_v = 2 * k + 1;

        H(row_u, 0) = u0_(j, i);
        H(row_v, 0) = v0_(j, i);
        for (int m = 0; m < M; ++m) {
            H(row_u, 1 + m) = rbfU_[m](j, i);
            H(row_v, 1 + m) = rbfV_[m](j, i);
        }
        z(row_u) = u_meas[k];
        z(row_v) = v_meas[k];

        float weight = (X_[j][i] < 10.0f) ? 0.5f : 1.0f;
        w(row_u) = weight;
        w(row_v) = weight;
    }
}

Eigen::VectorXf BaseflowCorrectionWindEstimator::solveRegularizedLS(const Eigen::MatrixXf &H,
                                                                    const Eigen::VectorXf &z,
                                                                    const Eigen::VectorXf &w,
                                                                    float lambda) const
{
    Eigen::VectorXf sqrtw = w.array().sqrt();
    Eigen::MatrixXf Hw = H;
    Eigen::VectorXf zw = z;
    for (int i = 0; i < H.rows(); ++i) {
        Hw.row(i) *= sqrtw(i);
        zw(i) *= sqrtw(i);
    }

    Eigen::MatrixXf reg = Eigen::MatrixXf::Identity(H.cols(), H.cols());
    reg(0, 0) = 0.0f;
    Eigen::MatrixXf A = Hw.transpose() * Hw + lambda * reg;
    Eigen::VectorXf b = Hw.transpose() * zw;
    return A.ldlt().solve(b);
}

void BaseflowCorrectionWindEstimator::estimateField(const std::vector<float> &meas_x,
                                                    const std::vector<float> &meas_y,
                                                    const std::vector<float> &u_meas,
                                                    const std::vector<float> &v_meas)
{
    U_field_.assign(ny_, std::vector<float>(nx_, 0.0f));
    V_field_.assign(ny_, std::vector<float>(nx_, 0.0f));
    W_field_.assign(ny_, std::vector<float>(nx_, 1.0f));

    if (u_meas.empty() || u_meas.size() != v_meas.size() || u_meas.size() != meas_x.size()) {
        for (int j = 0; j < ny_; ++j)
            for (int i = 0; i < nx_; ++i) {
                U_field_[j][i] = u0_(j, i);
                V_field_[j][i] = v0_(j, i);
            }
        return;
    }

    std::vector<int> idx(u_meas.size());
    std::vector<int> idy(u_meas.size());
    for (size_t k = 0; k < u_meas.size(); ++k)
        world_pos_to_grid_indices(meas_x[k], meas_y[k], idx[k], idy[k]);

    Eigen::MatrixXf H;
    Eigen::VectorXf z;
    Eigen::VectorXf w;
    buildDesignMatrix(idx, idy, u_meas, v_meas, H, z, w);

    float lambda = lambda_ * std::exp(-0.01f * static_cast<float>(u_meas.size()));
    Eigen::VectorXf theta = solveRegularizedLS(H, z, w, lambda);
    float scale = theta(0);
    int M = static_cast<int>(rbfU_.size());

    u_est_ = scale * u0_;
    v_est_ = scale * v0_;
    for (int m = 0; m < M; ++m) {
        float a = theta(1 + m);
        u_est_.noalias() += a * rbfU_[m];
        v_est_.noalias() += a * rbfV_[m];
    }

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            if (obsMask_(j, i) == 1.0f) {
                u_est_(j, i) = 0.0f;
                v_est_(j, i) = 0.0f;
            }
            U_field_[j][i] = u_est_(j, i);
            V_field_[j][i] = v_est_(j, i);
        }
    }
}
