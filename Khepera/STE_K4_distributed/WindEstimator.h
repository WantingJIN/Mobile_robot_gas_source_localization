#ifndef WIND_ESTIMATOR_H
#define WIND_ESTIMATOR_H

#include "constant_and_helper.h"

#include <Eigen/Dense>
#include <array>
#include <vector>

class WindEstimator {
public:
    WindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                  float **obstacle_list, int obs_nbr);
    virtual ~WindEstimator() = default;

    virtual void estimateField(const std::vector<float> &meas_x,
                               const std::vector<float> &meas_y,
                               const std::vector<float> &u_meas,
                               const std::vector<float> &v_meas) = 0;

    const std::vector<std::vector<float>> &getU() const { return U_field_; }
    const std::vector<std::vector<float>> &getV() const { return V_field_; }
    const std::vector<std::vector<float>> &getW() const { return W_field_; }

protected:
    float xmin_;
    float xmax_;
    float ymin_;
    float ymax_;
    float dx_;
    float dy_;
    int nx_;
    int ny_;
    std::vector<std::array<float, 4>> obstacles_;

    std::vector<std::vector<float>> X_;
    std::vector<std::vector<float>> Y_;
    std::vector<std::vector<float>> U_field_;
    std::vector<std::vector<float>> V_field_;
    std::vector<std::vector<float>> W_field_;

    void buildGrid();
    int grid_indices_to_idx(int i, int j) const { return i + nx_ * j; }
    void world_pos_to_grid_indices(float x, float y, int &i, int &j) const;
    int clamp_to_grid_x(int i) const;
    int clamp_to_grid_y(int j) const;
    int world_to_i(float x) const;
    int world_to_j(float y) const;
};

class GaussianWindEstimator : public WindEstimator {
public:
    GaussianWindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                          float **obstacles, int obs_nbr,
                          float sigma = 0.3f, float alpha = 1.0f, float c = 0.5f);

    void estimateField(const std::vector<float> &meas_x,
                       const std::vector<float> &meas_y,
                       const std::vector<float> &u_meas,
                       const std::vector<float> &v_meas) override;

private:
    void computeSDF();
    void computeNormals();
    void resetEstimateState();
    void addMeasurement(float meas_x, float meas_y, float u_meas, float v_meas);

    float sigma_;
    float alpha_;
    float c_;
    float U_global_;
    float V_global_;
    size_t processed_measurements_;
    float sum_u_;
    float sum_v_;
    std::vector<std::vector<float>> D_;
    std::vector<std::vector<float>> Nx_;
    std::vector<std::vector<float>> Ny_;
    std::vector<std::vector<float>> U_weighted_sum_;
    std::vector<std::vector<float>> V_weighted_sum_;
};

class BaseflowCorrectionWindEstimator : public WindEstimator {
public:
    BaseflowCorrectionWindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                                    float **obstacles, int obs_nbr,
                                    float rbf_length_scale = 1.0f, float lambda = 100.0f);

    void estimateField(const std::vector<float> &meas_x,
                       const std::vector<float> &meas_y,
                       const std::vector<float> &u_meas,
                       const std::vector<float> &v_meas) override;

private:
    int N_;
    Eigen::MatrixXf obsMask_;
    Eigen::MatrixXf phi_;
    Eigen::MatrixXf u0_;
    Eigen::MatrixXf v0_;
    Eigen::MatrixXf u_est_;
    Eigen::MatrixXf v_est_;
    float rbf_length_scale_;
    float lambda_;
    std::vector<Eigen::MatrixXf> rbfU_;
    std::vector<Eigen::MatrixXf> rbfV_;

    int idx1D(int i, int j) const { return j * nx_ + i; }
    void buildObstacleMask();
    void solvePotentialFlow();
    void computeVelocityFromPhi();
    void buildRbfCenters(std::vector<Eigen::Vector2f> &centers, float spacing_x, float spacing_y);
    void buildRbfBasis(const std::vector<Eigen::Vector2f> &centers, float length_scale);
    void buildDesignMatrix(const std::vector<int> &idx,
                           const std::vector<int> &idy,
                           const std::vector<float> &u_meas,
                           const std::vector<float> &v_meas,
                           Eigen::MatrixXf &H,
                           Eigen::VectorXf &z,
                           Eigen::VectorXf &w) const;
    Eigen::VectorXf solveRegularizedLS(const Eigen::MatrixXf &H,
                                       const Eigen::VectorXf &z,
                                       const Eigen::VectorXf &w,
                                       float lambda) const;
};

#endif
