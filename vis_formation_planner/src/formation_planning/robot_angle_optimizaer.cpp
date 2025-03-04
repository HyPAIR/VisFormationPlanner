#include "vis_formation_planner/robot_angle_optimizaer.h"
#include <iostream>
#include <stdexcept>

AngleOptimization::AngleOptimization(const std::vector<std::vector<std::pair<double, double>>>& intervals, 
                                     const std::vector<double>& initial_angles,
                                     const double& min_distance, 
                                     const double& max_distance, 
                                     const std::vector<double>& dis_scaler, 
                                     const Eigen::Vector2d& center, 
                                     const std::vector<Eigen::Vector2d>& points_prev)
    : robotIntervals(intervals), 
      previous_angles(initial_angles), 
      min_dis(min_distance),
      max_dis(max_distance),
      scaler(dis_scaler), 
      ctr(center),
      pts_prev(points_prev),
      numRobots(intervals.size()), 
      model(nullptr) {}

void AngleOptimization::optimize() {
    try {
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "angle_optimization.log");
        env.start();

        model = new GRBModel(env);

        // Initialize variables
        x.resize(numRobots);
        theta.resize(numRobots);
        cos_theta.resize(numRobots);
        sin_theta.resize(numRobots);
        d = model->addVar(0.0, 2 * M_PI, 0.0, GRB_CONTINUOUS, "d");

        for (size_t i = 0; i < numRobots; ++i) {
            size_t numIntervals = robotIntervals[i].size();
            x[i].resize(numIntervals);
            for (size_t j = 0; j < numIntervals; ++j) {
                x[i][j] = model->addVar(0, 1, 0, GRB_BINARY, "x_" + std::to_string(i) + "_" + std::to_string(j));
            }
            theta[i] = model->addVar(0.0, 2 * M_PI, 0.0, GRB_CONTINUOUS, "theta_" + std::to_string(i));
        }

        // if (!pts_prev.empty()) {
        //     for (size_t i = 0; i < numRobots; ++i) {
        //         cos_theta[i] = model->addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "cosTheta_" + std::to_string(i));
        //         model->addGenConstrCos(theta[i], cos_theta[i], "cos_theta_" + std::to_string(i));
        //         sin_theta[i] = model->addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "sinTheta_" + std::to_string(i));
        //         model->addGenConstrSin(theta[i], sin_theta[i], "din_theta_" + std::to_string(i));
        //     }
        // }

        // Add constraints
        for (size_t i = 0; i < numRobots; ++i) {
            GRBLinExpr sumIntervals = 0;
            for (size_t j = 0; j < robotIntervals[i].size(); ++j) {
                sumIntervals += x[i][j];
            }
            model->addConstr(sumIntervals == 1, "select_one_interval_" + std::to_string(i));
            for (size_t j = 0; j < robotIntervals[i].size(); ++j) {
                auto angle_diff = robotIntervals[i][j].second - robotIntervals[i][j].first;
                model->addConstr(theta[i] >= robotIntervals[i][j].first + angle_diff * 0.3 - (1 - x[i][j]) * 2 * M_PI,
                                 "theta_start_" + std::to_string(i) + "_" + std::to_string(j));
                model->addConstr(theta[i] <= robotIntervals[i][j].second - angle_diff * 0.3 + (1 - x[i][j]) * 2 * M_PI,
                                 "theta_end_" + std::to_string(i) + "_" + std::to_string(j));
            }
        }

        double targetAngle;
        if (numRobots > 2) {
            targetAngle = 2 * M_PI / numRobots;
        }
        else {
            targetAngle = M_PI / 2;
        }
        std::vector<GRBVar> angleDiffs(numRobots);
        for (size_t i = 0; i < numRobots; ++i) {
            size_t nextRobot = (i + 1) % numRobots;
            GRBVar rawDiff = model->addVar(-2 * M_PI, 2 * M_PI, 0.0, GRB_CONTINUOUS, "rawDiff_" + std::to_string(i));
            GRBVar wrappedDiff = model->addVar(0.0, 2 * M_PI, 0.0, GRB_CONTINUOUS, "wrappedDiff_" + std::to_string(i));

            model->addConstr(rawDiff == theta[nextRobot] - theta[i], "raw_diff_" + std::to_string(i));

            GRBVar absRawDiff = model->addVar(0.0, 2 * M_PI, 0.0, GRB_CONTINUOUS, "absRawDiff_" + std::to_string(i));
            GRBVar complementDiff = model->addVar(0.0, 2 * M_PI, 0.0, GRB_CONTINUOUS, "complementDiff_" + std::to_string(i));
            GRBVar isRawDiffSmaller = model->addVar(0, 1, 0, GRB_BINARY, "isRawDiffSmaller_" + std::to_string(i));

            model->addConstr(absRawDiff >= rawDiff, "abs_raw_pos_" + std::to_string(i));
            model->addConstr(absRawDiff >= -rawDiff, "abs_raw_neg_" + std::to_string(i));

            model->addConstr(complementDiff == 2 * M_PI - absRawDiff, "complement_diff_" + std::to_string(i));

            model->addQConstr(wrappedDiff == isRawDiffSmaller * absRawDiff + (1 - isRawDiffSmaller) * complementDiff,
                              "wrapped_diff_selection_" + std::to_string(i));

            angleDiffs[i] = wrappedDiff;
        }

        // Set objective: minimize deviation from target angle
        GRBQuadExpr objective = 0.0;
        for (size_t i = 0; i < numRobots; ++i) {
            objective += (angleDiffs[i] - targetAngle) * (angleDiffs[i] - targetAngle);
        }
        double penaltyWeight = 0.01;
        for (size_t i = 0; i < numRobots; ++i) {
            objective += penaltyWeight * (theta[i] - previous_angles[i]) * (theta[i] - previous_angles[i]);
        }
        // if (!pts_prev.empty()) {
        //     for (size_t i = 0; i < numRobots; ++i) {
        //         objective += penaltyWeight * (((ctr(0) + scaler[i] * ((min_dis + max_dis) / 2) * cos_theta[i]) - pts_prev[i](0)) * ((ctr(0) + scaler[i] * ((min_dis + max_dis) / 2) * cos_theta[i]) - pts_prev[i](0)) +
        //                      ((ctr(1) + scaler[i] * ((min_dis + max_dis) / 2) * sin_theta[i]) - pts_prev[i](1)) * ((ctr(1) + scaler[i] * ((min_dis + max_dis) / 2) * sin_theta[i]) - pts_prev[i](1)));
        //     }
        // }
        model->setObjective(objective, GRB_MINIMIZE);

        // Optimize
        model->optimize();
        if (model->get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            std::cerr << "Model is infeasible. Performing IIS analysis..." << std::endl;

            model->computeIIS();
            model->write("infeasible_model.ilp"); 

            throw std::runtime_error("Model is infeasible. Check infeasible_model.ilp for details.");
        }

    } catch (GRBException& e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Exception during optimization" << std::endl;
    }
}

std::vector<double> AngleOptimization::getTheta() const {
    std::vector<double> results(numRobots);
    for (size_t i = 0; i < numRobots; ++i) {
        results[i] = theta[i].get(GRB_DoubleAttr_X);
    }
    return results;
}

std::vector<int> AngleOptimization::getSelectedIntervals() const {
    std::vector<int> results(numRobots);
    for (size_t i = 0; i < numRobots; ++i) {
        for (size_t j = 0; j < robotIntervals[i].size(); ++j) {
            if (x[i][j].get(GRB_DoubleAttr_X) > 0.5) {
                results[i] = j;
                break;
            }
        }
    }
    return results;
}

double AngleOptimization::getDiversity() const {
    return d.get(GRB_DoubleAttr_X);
}