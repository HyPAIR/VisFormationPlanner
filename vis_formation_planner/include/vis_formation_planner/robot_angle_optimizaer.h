#ifndef ROBOT_ANGLE_OPTIMIZER_H
#define ROBOT_ANGLE_OPTIMIZER_H

#include <vector>
#include <utility>
#include <cmath>
#include <gurobi_c++.h>
// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

class AngleOptimization {
public:
    // Constructor
    AngleOptimization(const std::vector<std::vector<std::pair<double, double>>>& intervals, 
                      const std::vector<double>& initial_angles,
                      const double& min_distance, 
                      const double& max_distance, 
                      const std::vector<double>& dis_scaler, 
                      const Eigen::Vector2d& center, 
                      const std::vector<Eigen::Vector2d>& points_prev);

    // Optimize the angles
    void optimize();

    // Get results
    std::vector<double> getTheta() const;
    std::vector<int> getSelectedIntervals() const;
    double getDiversity() const;

private:
    size_t numRobots;
    std::vector<double> previous_angles;
    std::vector<std::vector<std::pair<double, double>>> robotIntervals;
    double min_dis;
    double max_dis; 
    std::vector<double> scaler; 
    Eigen::Vector2d ctr;
    std::vector<Eigen::Vector2d> pts_prev;
    std::vector<std::vector<GRBVar>> x;
    std::vector<GRBVar> theta;
    std::vector<GRBVar> cos_theta;
    std::vector<GRBVar> sin_theta;
    GRBVar d;
    GRBModel* model;
};

#endif // ROBOT_ANGLE_OPTIMIZER_H