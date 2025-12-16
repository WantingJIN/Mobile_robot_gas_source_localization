#ifndef GAUSSIAN_WIND_ESTIMATOR_H
#define GAUSSIAN_WIND_ESTIMATOR_H

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "constant_and_helper.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SuperLUSupport>

// ============================================================
// Base template class: WindEstimator (abstract interface)
// ============================================================
class WindEstimator {
public:
    WindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                  float ** obstacle_list, int obs_nbr)
        : xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax), nx_(nx), ny_(ny)
    {
        // Store obstacles
        for (int i = 0; i < obs_nbr; ++i) {
            std::array<float, 4> obs = {obstacle_list[i][0], obstacle_list[i][1],
                                        obstacle_list[i][2], obstacle_list[i][3]};
            obstacles_.push_back(obs);
        }
        dx_ = (xmax_ - xmin_) / (nx_ - 1); 
        dy_ = (ymax_ - ymin_) / (ny_ - 1);
        buildGrid();
    }

    virtual ~WindEstimator() = default;

    // Abstract methods to be implemented by derived classes
    virtual void estimateField(std::vector<float> meas_x, std::vector<float> meas_y, std::vector<double> u_meas, std::vector<double> v_meas) = 0;

    // Accessors
    const std::vector<std::vector<float>>& getU() const { return U_field_; }
    const std::vector<std::vector<float>>& getV() const { return V_field_; }
    const std::vector<std::vector<float>>& getW() const { return W_field_; }

protected:
    // Common geometry parameters
    float xmin_, xmax_, ymin_, ymax_;
    float dx_, dy_;
    int nx_, ny_;
    std::vector<std::array<float, 4>> obstacles_;

    // Common grid and field data
    std::vector<std::vector<float>> X_, Y_;
    std::vector<std::vector<float>> U_field_, V_field_, W_field_; 

    // Grid generation
    void buildGrid() {
        X_.resize(ny_, std::vector<float>(nx_));
        Y_.resize(ny_, std::vector<float>(nx_));
        for (int j = 0; j < ny_; ++j)
            for (int i = 0; i < nx_; ++i) {
                X_[j][i] = xmin_ + i * dx_;
                Y_[j][i] = ymin_ + j * dy_;
            }
    }
    // helper functions; convert between world position and (wind estimation) grid indices
    int grid_indices_to_idx(int i, int j) const { return i + nx_ * j; }
	void world_pos_to_grid_indices (double x, double y, int &i, int &j) const {
		i = clamp_to_grid_x(world_to_i(x));
		j = clamp_to_grid_y(world_to_j(y));
	}
	int clamp_to_grid_x(int i) const { return std::max(0, std::min(i, nx_ - 1)); }
	int clamp_to_grid_y(int j) const { return std::max(0, std::min(j, ny_ - 1)); }
	int grid_idx(int i, int j) const { return (i < 0 || i >= nx_ || j < 0 || j >= ny_) ? -1 : grid_indices_to_idx(i, j); }
	int world_to_i(double x) const { return (int)floor((x - source_bounds.x_min) / dx_); }
	int world_to_j(double y) const { return (int)floor((y - source_bounds.y_min) / dy_); }
};

// ============================================================
// Derived class: GaussianWindEstimator
// ============================================================
class GaussianWindEstimator : public WindEstimator {
public:
    GaussianWindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                  float ** obstacles, int obs_nbr, float sigma = 0.2, float alpha = 0.1, float c = 0.5);


    void estimateField(std::vector<float> meas_x, std::vector<float> meas_y, std::vector<double> u_meas, std::vector<double> v_meas) override;


private:
    void computeSDF();
    void computeNormals();
    float sigma_;
    float alpha_;
    float c_;
    const std::vector<std::vector<float>>& getSDF() const { return D_; }
    std::vector<std::vector<float>> D_;    // signed distance field
    std::vector<std::vector<float>> Nx_, Ny_; // normals from SDF
    void applyDivergenceFreeProjection(int max_iter = 500, float tol = 1e-4);
    float U_global_ = 0.0;
    float V_global_ = 0.0;
};

class BaseflowCorrectionWindEstimator : public WindEstimator {
public:
    BaseflowCorrectionWindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                  float ** obstacles, int obs_nbr, float RbfLengthScale = 1.0, double lambda = 4.0); 

    void estimateField(std::vector<float> meas_x, std::vector<float> meas_y, std::vector<double> u_meas, std::vector<double> v_meas) override;

private:
    // --- grid parameters ---
    int N_;                         // total number of grid points (nx_ * ny_)
    // --- obstacles / mask ---
    Eigen::MatrixXd obsMask_;      // ny_ x nx_ (0 or 1)

    // --- potential flow ---
    Eigen::MatrixXd phi_;          // ny_ x nx_
    Eigen::MatrixXd u0_, v0_;      // base flow

    // --- RBF basis ---
    float RbfLengthScale_;
    std::vector<Eigen::MatrixXd> rbfU_; // size M, each ny_ x nx_
    std::vector<Eigen::MatrixXd> rbfV_; // size M, each ny_ x nx_
    
    // --- regularization parameters ---
    double lambda_;
    // --- estimated field ---
    Eigen::MatrixXd u_est_, v_est_;

    // ==== helpers ====
    inline int idx1D(int i, int j) const { return j * nx_ + i; }

    void buildObstacleMask();
    void solvePotentialFlow();      // fills phi_
    void computeVelocityFromPhi();  // fills u0_, v0_

    void buildRbfCenters(std::vector<Eigen::Vector2d> &centers,
                         double spacingX, double spacingY);
    void buildRbfBasis(const std::vector<Eigen::Vector2d> &centers,
                       double lengthScale);

    void buildDesignMatrix(const std::vector<int> &idx, const std::vector<int> &idy,
                           const std::vector<double> &u_meas,
                           const std::vector<double> &v_meas,
                           Eigen::MatrixXd &H,
                           Eigen::VectorXd &z,
                           Eigen::VectorXd &w) const;

    Eigen::VectorXd solveRegularizedLS(const Eigen::MatrixXd &H,
                                       const Eigen::VectorXd &z,
                                       const Eigen::VectorXd &w,
                                       double lambda) const;
};

#endif
