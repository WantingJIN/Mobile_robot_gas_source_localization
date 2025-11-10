#ifndef GAUSSIAN_WIND_ESTIMATOR_H
#define GAUSSIAN_WIND_ESTIMATOR_H

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "constant_and_helper.h"

// ============================================================
// Base template class: WindEstimator (abstract interface)
// ============================================================
class WindEstimator {
public:
    WindEstimator(float xmin, float xmax, float ymin, float ymax, int nx, int ny,
                  float ** obstacle_list, int obs_nbr)
        : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax), nx(nx), ny(ny)
    {
        // Store obstacles
        for (int i = 0; i < obs_nbr; ++i) {
            std::array<float, 4> obs = {obstacle_list[i][0], obstacle_list[i][1],
                                        obstacle_list[i][2], obstacle_list[i][3]};
            obstacles.push_back(obs);
        }
        dx = (xmax - xmin) / (nx - 1); 
        dy = (ymax - ymin) / (ny - 1);
        buildGrid();
    }

    virtual ~WindEstimator() = default;

    // Abstract methods to be implemented by derived classes
    virtual void estimateField(int idx[], int idy[], std::vector<double> u_meas, std::vector<double> v_meas,
                               float sigma = 0.2, float alpha = 0.1, float c = 0.5) = 0;

    virtual void applyDivergenceFreeProjection(int max_iter = 500, float tol = 1e-4) = 0;

    // Accessors
    const std::vector<std::vector<float>>& getU() const { return U_field; }
    const std::vector<std::vector<float>>& getV() const { return V_field; }
    const std::vector<std::vector<float>>& getSDF() const { return D; }

protected:
    // Common geometry parameters
    float xmin, xmax, ymin, ymax;
    float dx, dy;
    int nx, ny;
    std::vector<std::array<float, 4>> obstacles;

    // Common grid and field data
    std::vector<std::vector<float>> X, Y;
    std::vector<std::vector<float>> D;    // signed distance field
    std::vector<std::vector<float>> Nx, Ny;
    std::vector<std::vector<float>> U_field, V_field, W_field; 
    float U_global_ = 0.0;
    float V_global_ = 0.0;

    // Grid generation
    void buildGrid() {
        X.resize(ny, std::vector<float>(nx));
        Y.resize(ny, std::vector<float>(nx));
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                X[j][i] = xmin + i * dx;
                Y[j][i] = ymin + j * dy;
            }
    }
};

// ============================================================
// Derived class: GaussianWindEstimator
// ============================================================
class GaussianWindEstimator : public WindEstimator {
public:
    GaussianWindEstimator(float xmin, float xmax, float ymin, float ymax, int Nx, int Ny,
                  float ** obstacles, int obs_nbr);

    void estimateField(int idx[], int idy[], std::vector<double> u_meas, std::vector<double> v_meas,
                               float sigma = 0.2, float alpha = 0.1, float c = 0.5) override;

    void calculateGlobalWind(float &U_global, float &V_global);

    void applyDivergenceFreeProjection(int max_iter = 500, float tol = 1e-4) override;

private:
    void computeSDF();
    void computeNormals();
};

#endif
