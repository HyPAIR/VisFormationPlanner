/**
* file prioritized_traj_optimization.hpp
* author Weijian Zhang (wxz163@student.bham.ac.uk)
* brief prioritized trajectory optimization using optimal control
* data 2023-11-22
* 
* @copyright Copyroght(c) 2023
*/
#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <map>
#include <random>
#include <cmath>
#include <unordered_set>
#include <stdlib.h>

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Submodules
#include <to_solver/corridor.hpp>
#include <to_solver/init_traj_planner.hpp>
#include <to_solver/mission.hpp>
#include <to_solver/param.hpp>

#include "com_fun.h"
#include "optimization.h"
#include "time.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

#include "vis_formation_planner/optimizer_interface.h"

using CppAD::AD;
typedef CppAD::AD<double> ADdouble;
using ADVector = std::vector<AD<double>>;

int qi;

double cost;

int diff_count;

int goal_theta_ind = 0;

int discrete = 1;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start;
size_t y_start;
size_t theta_start;
size_t yaw_start;
size_t v_start;
size_t omega_start;
size_t gimble_vel_start;
size_t vertice_start;

std::vector<ros::Publisher> way_point_pub;

std::vector<std::vector<std::array<int, 2>>> relative_pair; 
std::vector<std::vector<std::array<int, 2>>> relative_pair_updated; 
std::vector<std::array<int, 3>> selected_pair_new; 
std::vector<std::array<int, 3>> selected_pair_old;
std::vector<std::vector<int>> group;
std::vector<std::unordered_set<int>> robot_constraints;
std::vector<double> traj_length_set;
bool initial_traj_get = false;
int group_num;
int group_qn;
std::vector<double> robot_type;
// std::set<int> old_set;

struct Point {
    ADdouble x;
    ADdouble y;
};

struct Rectangle {
    ADdouble centerX;
    ADdouble centerY;
    ADdouble width;
    ADdouble height;
    ADdouble angle; 
};

void calculateRectangleVertices(const Rectangle& rect, Point vertices[4]) {
    ADdouble angleRad = rect.angle * M_PI / 180.0;

    ADdouble halfWidth = rect.width / 2.0;
    ADdouble halfHeight = rect.height / 2.0;

    ADdouble cosA = cos(angleRad);
    ADdouble sinA = sin(angleRad);

    vertices[0].x = rect.centerX + halfWidth * cosA + halfHeight * sinA; 
    vertices[0].y = rect.centerY - halfWidth * sinA + halfHeight * cosA; 
    vertices[1].x = rect.centerX - halfWidth * cosA + halfHeight * sinA; 
    vertices[1].y = rect.centerY + halfWidth * sinA + halfHeight * cosA; 
    vertices[2].x = rect.centerX - halfWidth * cosA - halfHeight * sinA; 
    vertices[2].y = rect.centerY + halfWidth * sinA - halfHeight * cosA; 
    vertices[3].x = rect.centerX + halfWidth * cosA - halfHeight * sinA; 
    vertices[3].y = rect.centerY - halfWidth * sinA - halfHeight * cosA; 
    }

    ADdouble calculateTriangleArea(const Point& A, const Point& B, const Point& C) {
    return 0.5 * CppAD::abs(A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y));
}

struct RobotCost {
    int index;
    double cost;

    RobotCost(int ind, double cost) : index(ind), cost(cost) {}
}; 

bool compareCost(const RobotCost& a, const RobotCost& b) {
    return a.cost > b.cost;
}

std::vector<RobotCost> costMap;
std::vector<double> robot_cost_set;

class FG_eval {
    public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    FG_eval(const std::vector<std::vector<double>>& human_traj,
            const std::vector<std::vector<std::vector<std::vector<double>>>>& constraint_set,
            std::vector<double> time_cons,
            std::vector<std::pair<int, double>> st_time_cons,
            std::vector<std::vector<std::vector<std::vector<double>>>> view_region_hyper_set,
            std::vector<std::vector<std::vector<double>>> opt_theta_set,
            std::vector<std::vector<int>> soft_sector_cons,
            std::vector<std::vector<Eigen::Vector2d>> mr_goal_set): 
                human_traj_(human_traj),
                constraint_set_(constraint_set),
                time_cons_(time_cons),
                st_time_cons_(st_time_cons),
                view_region_hyper_set_(view_region_hyper_set),
                opt_theta_set_(opt_theta_set),
                soft_sector_cons_(soft_sector_cons),
                mr_goal_set_(mr_goal_set)
                {}

    void operator()(ADvector& fg, const ADvector& vars) {
        // MPC implementation
        // fg a vector of constraints, x is a vector of constraints.
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        fg[0] = 0; 
        // diff_count = 0;
        // goal_theta_ind = 0;
        // for (int j = 0; j < group_qn; j++) {
        // qi = group[group_num][j];
        // if (robot_type[qi] < 1) {
        // diff_count++;
        // }
        // }
        for (int j = 0; j < group_qn; j++){
        int nu = group[group_num][j];
        ROS_INFO_STREAM("Try to solve Robot_" << nu);

        for (int i = 1; i < N - 1; i++) {
        fg[0] += 0.01 * W_X * CppAD::pow(vars[x_start + j * N + i] - plan[nu][i][0], 2);
        fg[0] += 0.01 * W_Y * CppAD::pow(vars[y_start + j * N + i] - plan[nu][i][1], 2); 
        }

        fg[0] += W_DV * CppAD::pow(vars[v_start + j*(N - 1)], 2);
        fg[0] += W_DOMEGA * CppAD::pow(vars[omega_start + j * (N - 1)], 2);

        // Minimize the value gap between sequential actuations.
        for (int i = 0; i < N - 2; i++) {
        fg[0] += 1.0 * CppAD::pow(vars[v_start + j * (N - 1) + i + 1] - vars[v_start + j * (N - 1)+ i], 2);
        fg[0] += 1.0 * CppAD::pow(vars[omega_start + j * (N - 1) + i + 1] - vars[omega_start + j * (N - 1) + i], 2);
        fg[0] += 1.0 * CppAD::pow(vars[gimble_vel_start + j*(N-1) + i + 1] - vars[gimble_vel_start + j*(N-1) + i], 2);
        }
        // for (int i = N - 30; i < N - 1; i++) {
        // fg[0] += W_FINAL * (CppAD::pow(vars[x_start + j*N + i + 1] - vars[x_start + j*N + i], 2) + 
        // CppAD::pow(vars[y_start + j*N + i + 1] - vars[y_start + j*N + i], 2)); 
        // }

        // Terminal constraints
        // fg[0] += W_FINAL * CppAD::pow(vars[x_start + j * N + N - 1] - plan[nu][N-1][0], 2);
        // fg[0] += W_FINAL * CppAD::pow(vars[y_start + j * N + N - 1] - plan[nu][N-1][1], 2); 
        // fg[0] += W_FINAL * CppAD::pow(vars[yaw_start + j * N + N - 1] - plan[nu][N-1][5], 2); 


        // Visibility Cost
        double prev_angle = atan2(human_traj_[0][1] - plan[nu][0][1], human_traj_[0][0] - plan[nu][0][0]);
        for(int i = 0; i < N; i++) {
            double length = std::hypot(human_traj_[i][1] - plan[nu][i][1], human_traj_[i][0] - plan[nu][i][0]);
            double new_angle = atan2((human_traj_[i][1] - plan[nu][i][1]) / length, (human_traj_[i][0] - plan[nu][i][0]) / length);
            while (new_angle - prev_angle > M_PI) {
                new_angle -= 2 * M_PI;  
            }
            while (new_angle - prev_angle < -M_PI) {
                new_angle += 2 * M_PI;
            }
            plan[nu][i][5] = new_angle;
            prev_angle = new_angle;
            }
            for (int i = 0; i < N; i++) {
                ADVector diff(2);
                diff[0] = vars[x_start + j * N + i] - opt_theta_set_[nu][i][1];
                diff[1] = vars[y_start + j * N + i] - opt_theta_set_[nu][i][2];
                //||p(tk) - qk||
                AD<double> norm_diff = CppAD::sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
                // (p(tk) - qk) · ξk
                AD<double> dot_product = diff[0] * opt_theta_set_[nu][i][3] + diff[1] * opt_theta_set_[nu][i][4];
                AD<double> dot_product_ = -diff[0] * opt_theta_set_[nu][i][3] -diff[1] * opt_theta_set_[nu][i][4];
                // ψocc
                AD<double> psi_occ = CppAD::cos(opt_theta_set_[nu][i][0]) - dot_product / norm_diff;
                fg[0] += W_VIS * CppAD::pow(psi_occ, 2);
                // fg[0] += W_GIMBLE * CppAD::pow(CppAD::cos(vars[yaw_start + j * N + i]) - dot_product_ / norm_diff, 2);
                fg[0] += 500.0 * CppAD::pow(vars[yaw_start + j * N + i] - plan[nu][i][5], 2);
            }

            // Angular Separation
            for (int i = 0; i < N; i++) {
                ADVector ego_diff(2);
                ADVector neighbor_diff(2);
                ego_diff[0] = vars[x_start + j * N + i] - opt_theta_set_[nu][i][1];
                ego_diff[1] = vars[y_start + j * N + i] - opt_theta_set_[nu][i][2];
                neighbor_diff[0] = plan[(nu + 1) % opt_theta_set_.size()][i][0] - opt_theta_set_[nu][i][1];
                neighbor_diff[1] = plan[(nu + 1) % opt_theta_set_.size()][i][1] - opt_theta_set_[nu][i][2];
                //||p(tk) − qk||
                AD<double> norm_ego_diff = CppAD::sqrt(ego_diff[0] * ego_diff[0] + ego_diff[1] * ego_diff[1]);
                //||pϕ(tk) − qk||
                AD<double> norm_neighbor_diff = CppAD::sqrt(neighbor_diff[0] * neighbor_diff[0] + neighbor_diff[1] * neighbor_diff[1]);
                // (p(tk) − qk) · (pϕ(tk) − qk)
                AD<double> dot_product = ego_diff[0] * neighbor_diff[0] + ego_diff[1] * neighbor_diff[1];
                AD<double> dot_product_ = -ego_diff[0] * neighbor_diff[0] - ego_diff[1] * neighbor_diff[1];
                // ψs
                double target_theta = opt_theta_set_.size() == 2 ? M_PI / 2 : 2 * M_PI / opt_theta_set_.size();
                AD<double> psi_seperate = CppAD::cos(target_theta) - dot_product / (norm_ego_diff * norm_neighbor_diff);
                fg[0] += 0.001 * CppAD::pow(psi_seperate, 2);
            }

            // visible regions soft constraints
            // for (int i = 0; i < soft_sector_cons_[nu].size(); i++) {
            // if (soft_sector_cons_[nu][i] == 1) {
            // int index = st_time_cons_[i].first;
            // fg[0] += W_SOFT * (CppAD::pow(vars[x_start + j * N + index] - mr_goal_set_[i][nu](0), 2) + 
            // CppAD::pow(vars[y_start + j * N + index] - mr_goal_set_[i][nu](1), 2));
            // }
            // }

            // // visible regions soft constraints
            // for (int i = 0; i < N; i++) {
            //   fg[0] += 0.001 * (CppAD::pow(vars[x_start + j * N + i] - mr_goal_set_[i][nu](0), 2) + 
            //                       CppAD::pow(vars[y_start + j * N + i] - mr_goal_set_[i][nu](1), 2));
            // }

            fg[1 + N * j + x_start] = vars[x_start + N * j];
            fg[1 + N * j + y_start] = vars[y_start + N * j];
            fg[1 + N * j + theta_start] = vars[theta_start + N * j];
            fg[1 + N * j + v_start] = vars[yaw_start + N * j];

            // The rest of the constraints
            // kinematic constraints
            for (int i = 0; i < N - 1; i++) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + N * j + i + 1];
            AD<double> y1 = vars[y_start + N * j + i + 1];
            AD<double> theta1 = vars[theta_start + N * j + i + 1];
            AD<double> yaw1 = vars[yaw_start + N * j + i + 1];

            // The state at time t.
            AD<double> x0 = vars[x_start + N * j + i];
            AD<double> y0 = vars[y_start + N * j + i];
            AD<double> theta0 = vars[theta_start + N * j + i];
            AD<double> yaw0 = vars[yaw_start + N * j + i];

            // Only consider the actuation at time t.
            AD<double> v0 = vars[v_start + (N-1) * j + i];
            AD<double> omega0 = vars[omega_start + (N-1) * j + i];
            AD<double> gimble0 = vars[gimble_vel_start + (N-1) * j + i];
            double dt = time_cons_[i];
            fg[2 + x_start + N * j + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * dt);
            fg[2 + y_start + N * j + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * dt);
            if (robot_type[nu] > 0) {
                fg[2 + theta_start + N * j + i] = theta1 - (theta0 + v0 * CppAD::tan(omega0) / 0.65 * dt);
            }
            else {
                fg[2 + theta_start + N * j + i] = theta1 - (theta0 + omega0 * dt);
            }
            fg[2 + v_start + N * j + i] = yaw1 - (yaw0 + gimble0 * dt);
            } 
        }
        int cont = N * 4 * group_qn + 1;

        // safe inter-distance constraints
        for (int j = 0; j < group_qn; j++){
            int nu = group[group_num][j];
            for (int i = 0; i < N; i++) {
                fg[cont++] = CppAD::pow(vars[x_start + N * j + i] - human_traj_[i][0], 2) + CppAD::pow(vars[y_start + N * j + i] - human_traj_[i][1], 2);
            }
        } 

        int neww, old, tt;
        // selected_pair_new: <timestep, robot_i, robot_j>
        for (int i = 0; i < selected_pair_new.size(); i++) {
            tt = selected_pair_new[i].at(0);
            for (int k = 0; k < group_qn; k++) {
                if(group[group_num][k] == selected_pair_new[i].at(1))
                neww = k;
                else if (group[group_num][k] == selected_pair_new[i].at(2))
                old = k;
                } 
                fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - vars[x_start + N * old + tt], 2)
                + CppAD::pow(vars[y_start + N * neww + tt] - vars[y_start + N * old + tt], 2);
                cont++;
        } 
        for (int i = 0; i < selected_pair_old.size(); i++) {
            tt = selected_pair_old[i].at(0);
            for (int k = 0; k < group_qn; k++){
                if (group[group_num][k] == selected_pair_old[i].at(1)){
                    neww = k;
                    break;
                }
            }

            fg[cont] = CppAD::pow(vars[x_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][0], 2)
            +CppAD::pow(vars[y_start + N * neww + tt] - plan[selected_pair_old[i].at(2)][tt][1], 2);
            cont++;
        }

        // safe corridors constraints
        for (int j = 0; j < group_qn; j++){
            int nu = group[group_num][j];
            for (int i = 0; i < constraint_set_[nu].size(); i++) {
                for (int k = 0; k < constraint_set_[nu][i].size(); k++) {
                    fg[cont++] = vars[x_start + N * j + i] * constraint_set_[nu][i][k][0] + vars[y_start + N * j + i] * constraint_set_[nu][i][k][1] - constraint_set_[nu][i][k][2];
                }
            }
        }
        // visible regions constraints
        for (int j = 0; j < group_qn; j++){
            int nu = group[group_num][j];
            for (int i = 0; i < view_region_hyper_set_[nu].size(); i++) {
                for (int k = 0; k < view_region_hyper_set_[nu][i].size(); k++) {
                    int index = st_time_cons_[i].first;
                    if (soft_sector_cons_[nu][i] == 1) {
                        fg[cont++] = -100;
                    }
                    else {
                        fg[cont++] = vars[x_start + N * j + index] * view_region_hyper_set_[nu][i][k][0] + vars[y_start + N * j + index] * view_region_hyper_set_[nu][i][k][1] - view_region_hyper_set_[nu][i][k][2];
                    }
                }
            }
        }
        // for (int j = 0; j < group_qn; j++) { 
        // for (int i = 0; i < N; j++) {
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8] - vars[x_start + N * j + i] - 0.4 * CppAD::cos(vars[theta_start + N * j + i]) + 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 1] - vars[x_start + N * j + i] - 0.4 * CppAD::sin(vars[theta_start + N * j + i]) - 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 2] - vars[x_start + N * j + i] + 0.4 * CppAD::cos(vars[theta_start + N * j + i]) + 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 3] - vars[x_start + N * j + i] + 0.4 * CppAD::sin(vars[theta_start + N * j + i]) - 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 4] - vars[x_start + N * j + i] - 0.4 * CppAD::cos(vars[theta_start + N * j + i]) - 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 5] - vars[x_start + N * j + i] - 0.4 * CppAD::sin(vars[theta_start + N * j + i]) + 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 6] - vars[x_start + N * j + i] + 0.4 * CppAD::cos(vars[theta_start + N * j + i]) - 0.5 * CppAD::sin(vars[theta_start + N * j + i]);
        // fg[cont++] = vars[vertice_start + (j * N + i) * 8 + 7] - vars[x_start + N * j + i] + 0.4 * CppAD::sin(vars[theta_start + N * j + i]) + 0.5 * CppAD::cos(vars[theta_start + N * j + i]);
        // }
        // }
    }
    private:
    std::vector<std::vector<double>> human_traj_;
    std::vector<std::vector<std::vector<std::vector<double>>>> constraint_set_;
    std::vector<double> time_cons_;
    std::vector<std::pair<int, double>> st_time_cons_;
    std::vector<std::vector<std::vector<std::vector<double>>>> view_region_hyper_set_;
    std::vector<std::vector<std::vector<double>>> opt_theta_set_;
    std::vector<std::vector<int>> soft_sector_cons_;
    std::vector<std::vector<Eigen::Vector2d>> mr_goal_set_;
};

class MPCPlanner {
    public:
    std_msgs::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;
    std::vector<double> robot_w_list;

    MPCPlanner(
    std::vector<std::vector<double>> _human_traj,
    std::vector<std::vector<std::vector<std::vector<double>>>> _constraint_set,
    std::vector<double> _time_cons,
    std::vector<std::pair<int, double>> _st_time_cons,
    std::vector<std::vector<std::vector<std::vector<double>>>> _view_region_hyper_set,
    std::vector<std::vector<std::vector<double>>> _opt_theta_set,
    std::vector<std::vector<Eigen::Vector3d>> _ref_paths,
    // std::shared_ptr<Corridor> _corridor_obj,
    // std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
    SwarmPlanning::Param _param,
    std::vector<double>& _theta_set,
    std::vector<std::vector<int>>& _soft_sector_cons,
    std::vector<std::vector<Eigen::Vector2d>>& _mr_goal_set):
    human_traj(std::move(_human_traj)),
    constraint_set(std::move(_constraint_set)),
    time_cons(std::move(_time_cons)),
    st_time_cons(std::move(_st_time_cons)),
    view_region_hyper_set(std::move(_view_region_hyper_set)),
    opt_theta_set(std::move(_opt_theta_set)),
    ref_paths(std::move(_ref_paths)),
    // corridor_obj(std::move(_corridor_obj)),
    // initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
    param(std::move(_param)),
    theta_set(_theta_set),
    soft_sector_cons(_soft_sector_cons),
    mr_goal_set(_mr_goal_set)

    {

    outdim = 3; // the number of outputs (x,y,z)

    // T = initTrajPlanner_obj.get()->T;
    initTraj.resize(ref_paths.size());
    for (int i = 0; i < ref_paths.size(); i++) {
        for (int j = 0; j < ref_paths[i].size(); j++) {
            initTraj[i].emplace_back(octomap::point3d(ref_paths[i][j][0],
            ref_paths[i][j][1],
            ref_paths[i][j][2]));
        }
    }
    for(int i = 0; i < ref_paths[0].size(); i++){
        T.emplace_back(i * param.time_step);
    }
    M = T.size()-1; // the number of segments

    // initTraj = initTrajPlanner_obj.get()->initTraj;
    // SFC = corridor_obj.get()->SFC;

    int i,j,k;

    double dx;
    double dy;
    double dis;
    qn = ref_paths.size();
    for (i = 0; i < qn; i++){
        for(j = 0;j < M; j++){

            dx = (initTraj[i][j+1].x() - initTraj[i][j].x()) / discrete;
            dy = (initTraj[i][j+1].y() - initTraj[i][j].y()) / discrete;
            dis = discrete * discrete * (dx*dx + dy*dy);

            for (k = 0; k < discrete; k++){
                plan[i][discrete*j+k][0] = initTraj[i][j].x() + dx*k;
                plan[i][discrete*j+k][1] = initTraj[i][j].y() + dy*k;
                // 每个时刻的速度
                if (dis < 0.1) {
                    plan[i][discrete*j+k][3] = 0;
                }
                if (dis<1.1){
                    plan[i][discrete*j+k][3] = MAXV*0.7;
                }
                else{
                    plan[i][discrete*j+k][3] = MAXV;
                }
            }
        }
        plan[i][discrete*M][0] = initTraj[i][M].x();
        plan[i][discrete*M][1] = initTraj[i][M].y();
    }

    N = M*discrete+1;
    // 配准朝向角
    for (int qi = 0; qi < qn; qi++) {
        for (int next = discrete; next < N; next += discrete){
            if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
                continue;
            else{
                plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
                break;
            }
        }
    }

    for (int qi = 0; qi < qn; qi++) {
        for (int next = N - 1 - discrete; next < N; next -= discrete){
            if ((plan[qi][next][1] == plan[qi][N-1][1]) && (plan[qi][next][0] == plan[qi][N-1][0]))
                continue;
            else{
                plan[qi][N-1][2] = atan2(plan[qi][N-1][1] - plan[qi][next][1], plan[qi][N-1][0] - plan[qi][next][0]);
            break;
            }
        }
    }

    double angle;

    for (i = 0; i < qn; i++){
        for(j = 0; j < M; j++){

            dx = initTraj[i][j+1].x() - initTraj[i][j].x();
            dy = initTraj[i][j+1].y() - initTraj[i][j].y();
            dis = dx*dx+dy*dy;
            if (j > 0){
                if (dis > 0.1){
                angle = atan2(dy, dx);
                if (angle - plan[i][discrete*(j-1)][2] > PI)
                    angle = angle - 2 * PI;
                else if(plan[i][discrete*(j-1)][2] - angle > PI)
                    angle = angle + 2 * PI;
                }
                else angle = plan[i][discrete*(j-1)][2];
            }
            else {
                angle = plan[i][0][2];
            }

            for (k = 0; k < discrete; k++){
                if (angle < -M_PI) {
                    while (angle < -M_PI) {
                        angle += 2 * M_PI;
                    }
                }
                else if (angle > M_PI) {
                    while (angle > M_PI) {
                        angle -= 2 * M_PI;
                    }
                } 
                plan[i][discrete*j + k][2] = angle;
            }
        }
    }
    // plan[~][~][4]存放角速度
    for (i = 0; i < qn; i++){
        double dt = time_cons[i];
        for(j = 0; j < M; j++){
            for (k = 0; k < discrete; k++){
                plan[i][discrete*j+k][4] = (plan[i][discrete*(j+1)][2] - plan[i][discrete*j][2])*(1.0/(discrete * 1.0)) / dt;
            }
        }
    }

    for (i = 0; i < qn; i++){
        double dt = time_cons[i];
        for(j = 0; j < M; j++){
            for (k = 0; k < discrete; k++){
                plan[qi][discrete*j+k][5] = atan2(opt_theta_set[qi][discrete*j][2] - plan[qi][discrete*j][1], opt_theta_set[qi][discrete*j][1] - plan[qi][discrete*j][0]);
            }
        }
    }

    for (i = 0; i < qn; i++){
        double dt = time_cons[i];
        for(j = 0; j < M; j++){
            for (k = 0; k < discrete; k++){
                plan[i][discrete*j+k][6] = (plan[i][discrete*(j+1)][5] - plan[i][discrete*j][5])*(1.0/(discrete * 1.0)) / dt;
            }
        }
    }

    for (i = 0; i < qn; i++) {
        plan[i][N-1][2] = 0;
    }

    theta_set.resize(plan.size());
    for (int i = 0; i < plan.size(); i++) { 
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(-M_PI, M_PI);
        double random_number = dist(gen);
        theta_set[i] = random_number;
        // plan[i][N-1][2] = random_number;
        plan[i][N-1][2] = 0.0;
    }
    for (i = 0; i < qn; i++) {
        double robot_w = 0;
        for (j = 0; j < plan[i][4].size(); j++) {
            robot_w += fabs(plan[i][4][j]);
        }
        robot_w_list.push_back(robot_w);
    }

    }

    bool UpdateConflictPair(std::vector<std::vector<std::vector<double>>> plan, 
    std::vector<std::vector<std::array<int, 2>>>& relative_pair_updated, const int& current_ind, int& current_ind_conflict) {
    bool is_collide = false;
    relative_pair_updated.clear();
    std::array<int, 2> sa; // 存放2个int元素的数组
    std::vector <std::array<int, 2>> sb;

    double threshold = 0;
    bool first_conflict_found = false;
    // if (qn <= 8)
    // threshold = pow(2 * mission.quad_size[0], 2);
    // else
    // threshold = pow(2 * mission.quad_size[0], 2);
    for (int k = 0; k < N; k++){
        sb.clear();
        // 遍历所有机器人对，找到小于阈值的对(i, j), i << j
        for(int i = 0; i < qn; i++){
            for (int j = i + 1; j < qn; j++) {
            // double dis = pow(plan[i][k][0] - plan[j][k][0], 2.0) + pow(plan[i][k][1] - plan[j][k][1], 2.0);
            // Rectangle Rectangle_1 = {plan[i][k][0], plan[i][k][1] , 0.8, 1.0, plan[i][k][2]}; 
            // Rectangle Rectangle_2 = {plan[j][k][0], plan[j][k][1] , 0.8, 1.0, plan[j][k][2]}; 
            // Point vertices_1[4], vertices_2[4];
            // calculateRectangleVertices(Rectangle_1, vertices_1);
            // calculateRectangleVertices(Rectangle_2, vertices_2);
            // for (int rec_1_ind = 0; rec_1_ind < 4; ++rec_1_ind) {
            // ADdouble area = 0.0;
            // Point A, B, C;
            // for (int rec_2_ind = 0; rec_2_ind < 3; ++rec_2_ind) {
            // A = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
            // B = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
            // C = {vertices_2[rec_2_ind+1].x, vertices_2[rec_2_ind+1].y};
            // area += calculateTriangleArea(A, B, C);
            // }
            // B = {vertices_2[0].x, vertices_2[0].y};
            // C = {vertices_2[3].x, vertices_2[3].y};
            // area += calculateTriangleArea(A, B, C);
            // if (area <= 1.2) {
            // is_collide = true;
            // break;
            // }
            // }
            if (hypot(plan[i][k][0] - plan[j][k][0], plan[i][k][1] - plan[j][k][1]) < 1.2) {
                is_collide = true;
                first_conflict_found = true;
                return first_conflict_found;
                break; 
            }
            // if (!is_collide) {
            // for (int rec_2_ind = 0; rec_2_ind < 4; ++rec_2_ind) {
            // ADdouble area = 0.0;
            // Point A, B, C;
            // for (int rec_1_ind = 0; rec_1_ind < 3; ++rec_1_ind) {
            // A = {vertices_2[rec_2_ind].x, vertices_2[rec_2_ind].y};
            // B = {vertices_1[rec_1_ind].x, vertices_1[rec_1_ind].y};
            // C = {vertices_1[rec_1_ind+1].x, vertices_1[rec_1_ind+1].y};
            // area += calculateTriangleArea(A, B, C);
            // }
            // B = {vertices_1[0].x, vertices_1[0].y};
            // C = {vertices_1[3].x, vertices_1[3].y};
            // area += calculateTriangleArea(A, B, C);
            // if (area <= 1.2) {
            // is_collide = true;
            // break;
            // }
            // }
            // }
            // if (is_collide) {
            // sb.emplace_back(sa = {i, j});
            // if (i == current_ind && !first_conflict_found) {
            // robot_constraints[i].emplace(j);
            // robot_constraints[j].emplace(i);
            // current_ind_conflict = j;
            // first_conflict_found = true;
            // }
            // is_collide = false;
            // }
            }
        } 
        relative_pair_updated.emplace_back(sb);
    } 
    return first_conflict_found;
    }

    void UpdateCost(const std::vector<std::vector<std::vector<double>>>& plan, const std::vector<double>& robot_cost_set,
    const int& current_ind) {
        int costMapLength;
        if (!initial_traj_get) {
            for (int i = 0; i < plan.size(); i++) {
                for (int j = 1; j < N; j++) {
                    traj_length_set[i] += hypot(plan[i][j][0] - plan[i][j-1][0], plan[i][j][1] - plan[i][j-1][1]);
                }
            }
        }
        else {
        for (int i = 1; i < N; i++) {
            traj_length_set[current_ind] += hypot(plan[current_ind][i][0] - plan[current_ind][i-1][0], plan[current_ind][i][1] - plan[current_ind][i-1][1]);
        } 
        }
        double traj_length_all = 0.0, cost_total = 0.0;
        for (int i = 0; i < traj_length_set.size(); i++) {
            traj_length_all += traj_length_set[i];
            cost_total += robot_cost_set[i];
        }
        if (initial_traj_get) {
            costMapLength = costMap.size();
        }
        else {
            costMapLength = plan.size();
        }
        for (int i = 0; i < costMapLength; i++) {
            int newIndex;
            if (initial_traj_get) {
                newIndex = costMap[i].index;
            }
            else {
                newIndex = i;
            }
            auto it = std::find_if(costMap.begin(), costMap.end(), [newIndex](const RobotCost& robot_cost) { return robot_cost.index == newIndex; });
            if (it != costMap.end()) {
                costMap.erase(it);
            }
            costMap.emplace_back(newIndex, traj_length_set[newIndex]/traj_length_all + robot_cost_set[newIndex]/cost_total + robot_type[newIndex]);
        }
        std::sort(costMap.begin(), costMap.end(), compareCost);
        initial_traj_get = true;

    }

    bool Solve(int index,std::vector<std::vector<int>>& soft_sector_cons, std::vector<std::vector<int>>& refined_index_set) {
        bool ok = true;
        // vector<double>
        typedef CPPAD_TESTVECTOR(double) Dvector;
        // diff_count = 0;
        // for (int j = 0; j < group_qn; j++) {
        // qi = group[group_num][j];
        // if (robot_type[qi] < 1) {
        // diff_count++;
        // }
        // }
        // Set the number of model variables (includes both states and inputs).
        // For example: If the state is a 4 element vector, the actuators is a 2
        // element vector and there are 10 timesteps. The number of variables is:
        int corridor_size = 0;
        int view_region_size = 0;
        for (int i = 0; i < group_qn; i++) {
            qi = group[group_num][i];
            for (int j = 0; j < constraint_set[qi].size(); j++) {
                corridor_size += constraint_set[qi][j].size();
            }
        }
        for (int i = 0; i < group_qn; i++) {
            qi = group[group_num][i];
            for (int j = 0; j < view_region_hyper_set[qi].size(); j++) {
                view_region_size += view_region_hyper_set[qi][j].size();
            }
        }
        size_t n_vars = (N * 4 + (N - 1) * 4) * group_qn;
        // size_t n_vars = (N * 3 + (N - 1) * 2) * group_qn + N * 4 * 2 * group_qn;
        // Set the number of constraints
        // size_t n_constraints = N * 3 * group_qn + 2 * group_qn + 1 * diff_count;
        size_t n_constraints = N * 4 * group_qn;
        size_t inter_dis_constraints = N * group_qn;
        // size_t all_constraints = n_constraints + qn*(qn-1)*int(N/2)/2;
        size_t pair_constraints = selected_pair_new.size() + selected_pair_old.size();
        size_t all_constraints = n_constraints + inter_dis_constraints + pair_constraints + corridor_size + view_region_size;
        // Initial value of the independent variables.
        // SHOULD BE 0 besides initial state.
        Dvector vars(n_vars);

        Dvector vars_lowerbound(vars);
        Dvector vars_upperbound(vars);
        // Set lower and upper limits for variables.
        for (int i = 0; i < v_start; i++) {
            vars_lowerbound[i] = -BOUND;
            vars_upperbound[i] = BOUND;
        }

        // diff_count = 0;
        // for (int j = 0; j < group_qn; j++) {
        // qi = group[group_num][j];
        // if (robot_type[qi] < 1) {
        // diff_count++;
        // }
        // }
        vertice_start = (N * 3 + (N - 1) * 2) * group_qn;
        for (int j = 0; j < group_qn; j++){
            qi = group[group_num][j];
            for (int i = 0; i < N; i++) {
                // initial guess using initial states
                vars[x_start + j * N + i] = plan[qi][i][0];
                vars[y_start + j * N + i] = plan[qi][i][1];
                vars[theta_start + j * N + i] = plan[qi][i][2];
                vars[yaw_start + j * N + i] = plan[qi][i][5];

                // initial guess using initial control
                if (i < N-1){
                    vars[v_start + j * (N - 1) + i] = plan[qi][i][3];
                    vars[omega_start + j * (N -1) + i] = plan[qi][i][4];
                    vars[gimble_vel_start + j * (N -1) + i] = plan[qi][i][6];
                }
                // if ( i > 1 && i == constraint_set[qi][count + 1].second && count < constraint_set[j].size() - 1){
                // count++;
                // }

                vars_lowerbound[x_start + j * N + i] = -BOUND;
                vars_lowerbound[y_start + j * N + i] = -BOUND;
                vars_lowerbound[yaw_start + j * N + i] = -BOUND;
                vars_upperbound[x_start + j * N + i] = BOUND;
                vars_upperbound[y_start + j * N + i] = BOUND;
                vars_upperbound[yaw_start + j * N + i] = BOUND;
            }
        }
        for (int j = 0; j < group_qn; j++) {
            qi = group[group_num][j];
            for (int i = v_start + j * (N - 1); i < v_start + (j + 1) * (N - 1); i++) {
                if (robot_type[qi] > 0) {
                    vars_lowerbound[i] = 0;
                    vars_upperbound[i] = MAXV;
                    if (param.backward_enable)
                        vars_lowerbound[i] = -MAXV;
                }
                else {
                    vars_lowerbound[i] = 0;
                    vars_upperbound[i] = MAXV;
                    if (param.backward_enable)
                        vars_lowerbound[i] = -MAXV; 
                }
            }
        }

        for (int j = 0; j < group_qn; j++) {
            qi = group[group_num][j];
            for (int i = omega_start + j * (N - 1); i < omega_start + (j + 1) * (N - 1); i++) {
                if (robot_type[qi] > 0) {
                    vars_upperbound[i] = MAXPHI;
                    vars_lowerbound[i] = -MAXPHI; 
                }
                else {
                    vars_upperbound[i] = MAXOMEGA;
                    vars_lowerbound[i] = -MAXOMEGA;
                }
            }
        }

        for (int j = 0; j < group_qn; j++) {
            qi = group[group_num][j];
            for (int i = gimble_vel_start + j * (N - 1); i < gimble_vel_start + (j + 1) * (N - 1); i++) {
                vars_upperbound[i] = MAXGIMBLE;
                vars_lowerbound[i] = -MAXGIMBLE; 
            }
        }

        // Lower and upper limits for the constraints
        // Should be 0 besides initial state.
        Dvector constraints_lowerbound(all_constraints);
        Dvector constraints_upperbound(all_constraints);
        for (int i = 0; i < n_constraints; i++) {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        for (int i = n_constraints; i < n_constraints + inter_dis_constraints; i++) {
            constraints_lowerbound[i] = 1;
            constraints_upperbound[i] = BOUND;
        }
        // 0.2 to 0.3: collsion avoidance distance
        // double inter_collision = 2 * mission.quad_size[0] + 0.2;
        // inter_collision = inter_collision * inter_collision;
        for (int i = n_constraints + inter_dis_constraints; i < n_constraints + inter_dis_constraints + pair_constraints; i++)
        {
            // constraints_lowerbound[i] = inter_collision;
            constraints_lowerbound[i] = 1.5;
            constraints_upperbound[i] = BOUND;
        }

        for (int i = n_constraints + inter_dis_constraints + pair_constraints; i < all_constraints; i++)
        {
            // constraints_lowerbound[i] = inter_collision;
            constraints_lowerbound[i] = -BOUND;
            constraints_upperbound[i] = -1.0e-20;
        }

        for (int j = 0; j < group_qn; j ++){
            qi = group[group_num][j];
            double startangle;
            for (int next = discrete; next < N; next += discrete){
                if ((plan[qi][next][1] == plan[qi][0][1]) & (plan[qi][next][0] == plan[qi][0][0]))
                    continue;
                else{
                    startangle = atan2(plan[qi][next][1] - plan[qi][0][1], plan[qi][next][0] - plan[qi][0][0]);
                    break;
                }
            }
            // start position constraints
            constraints_lowerbound[x_start + N * j] = plan[qi][0][0];
            constraints_lowerbound[y_start + N * j] = plan[qi][0][1];
            constraints_upperbound[x_start + N * j] = plan[qi][0][0];
            constraints_upperbound[y_start + N * j] = plan[qi][0][1];

            // if(param.initial_angle)
            // {
            // constraints_lowerbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
            // constraints_upperbound[theta_start + N * j] = mission.startState[qi][2] * PI / 2;
            // }
            // else
            // { 
            constraints_lowerbound[theta_start + N * j] = startangle;
            constraints_upperbound[theta_start + N * j] = startangle;
            constraints_lowerbound[v_start + N * j] = plan[qi][0][5];
            constraints_upperbound[v_start + N * j] = plan[qi][0][5];
            // }

        }

        // object that computes objective and constraints
        FG_eval fg_eval(human_traj, constraint_set, time_cons, st_time_cons, view_region_hyper_set, opt_theta_set, soft_sector_cons, mr_goal_set);
        // options for IPOPT solver
        std::string options;

        options += "Numeric tol 1e-5\n";
        options += "String linear_solver mumps\n";
        // Uncomment this if you'd like more print information
        options += "Integer print_level 0\n";
        // NOTE: Setting sparse to true allows the solver to take advantage
        // of sparse routines, this makes the computation MUCH FASTER. If you
        // can uncomment 1 of these and see if it makes a difference or not but
        // if you uncomment both the computation time should go up in orders of
        // magnitude.
        options += "Sparse true forward\n";
        options += "Sparse true reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;

        // solve the problem
        CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

        // Check some of the solution values
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
        if (!ok) {
            std::cerr << "IPOPT Infeasibility Detected!" << std::endl;
            std::cerr << "Checking Violated Constraints in Range [" 
                    << n_constraints + inter_dis_constraints + pair_constraints + corridor_size
                    << " - " << all_constraints - 1 << "]" << std::endl;
            // for (size_t i = n_constraints + inter_dis_constraints + pair_constraints + corridor_size; i < all_constraints; i++) {
            for (size_t i = 0; i < all_constraints; i++) {
                if (solution.g[i] < constraints_lowerbound[i] - 0.0001 || solution.g[i] > constraints_upperbound[i] + 0.0001) {
                int find_ind = 0;
                int violated_ind = i - (n_constraints + inter_dis_constraints + pair_constraints + corridor_size);
                for (int i_ = 0; i_ < group_qn; i_++) {
                    qi = group[group_num][i_];
                    for (int j_ = 0; j_ < view_region_hyper_set[qi].size(); j_++) {
                        find_ind += view_region_hyper_set[qi][j_].size();
                        if (find_ind > violated_ind) {
                            // soft_sector_cons[qi][j_-1] = 1;
                            // refined_index_set[qi].push_back(j_-1);
                            continue;
                        }
                    }
                }
                std::cerr << "Visibility Constraint [" << i << "] Violated: " << solution.g[i] 
                        << " (Expected Bounds: [" << constraints_lowerbound[i] 
                        << ", " << constraints_upperbound[i] << "])" << std::endl;
                }
            }
        }

        cost += solution.obj_value;

        if(!ok){
            ROS_INFO_STREAM("Infeasible Solution: Group_" << group_num);
            return false;
        }

        for (int j = 0; j < group_qn; j++){
            qi = group[group_num][j];
            for (int i = 0; i < N; i++){
                plan[qi][i][0] = solution.x[x_start + j * N + i];
                plan[qi][i][1] = solution.x[y_start + j * N + i];
                plan[qi][i][2] = solution.x[theta_start + j * N + i];
                plan[qi][i][5] = solution.x[yaw_start + j * N + i];
            }
            for (int i = 0; i < N - 1; i++) {
                plan[qi][i][3] = solution.x[v_start + j * (N - 1) + i];
                plan[qi][i][4] = solution.x[omega_start + j * (N - 1) + i];
                plan[qi][i][6] = solution.x[gimble_vel_start + j * (N - 1) + i];
            }
        }
        robot_cost_set[index] = solution.obj_value;
        return true;
    }

    // selected_pair_old是高优先级的机器人吗？ 是:)
    bool update(bool log, std::vector<std::vector<std::vector<double>>>& plan_set, std::vector<std::vector<int>>& soft_sector_cons, std::vector<std::vector<int>>& refined_index_set){ 
        int corridor_size = 0;
        Timer timer;
        timer.reset();
        for (int i = 0; i < group_qn; i++) {
        qi = group[group_num][i];
            for (int j = 0; j < constraint_set[qi].size(); j++) {
                corridor_size += constraint_set[qi][j].size();
            }
        }
        qn = plan_set.size();
        initial_traj_get = false;
        robot_cost_set.resize(qn, 0.0);
        robot_constraints.resize(qn);
        traj_length_set.resize(plan.size(), 0.0);
        Timer timer_step;
        // for (int i = 0; i < mission.quad_type.size(); i++) {
        // robot_type.push_back(mission.quad_type[i]);
        // }
        for (int i = 0; i < plan_set.size(); i++) {
            robot_type.push_back(0);
        }
        geometry_msgs::PoseStamped pp;
        group.clear();
        selected_pair_old.clear();
        // optimization without coordination
        for (int i = 0; i < qn; i++) {
            group.push_back({i});
        }
        group_qn = 1;
        for (int i = 0; i < group.size(); i++) {
            group_num = i;
            int temp_qn = group[i].size();
            x_start = 0;
            y_start = x_start + temp_qn * N;
            theta_start = y_start + temp_qn * N;
            v_start = theta_start + temp_qn * N;
            omega_start = v_start + temp_qn * (N - 1); 
            yaw_start = omega_start + temp_qn * N;
            gimble_vel_start = yaw_start + temp_qn * (N - 1); 
            timer.stop();
            if (!Solve(group[i][0], soft_sector_cons, refined_index_set)) {
                return false;
            }
        }
        for (int i = 0; i < group.size(); i++) {
            group_num = i;
            int temp_qn = group[i].size();
            x_start = 0;
            y_start = x_start + temp_qn * N;
            theta_start = y_start + temp_qn * N;
            v_start = theta_start + temp_qn * N;
            omega_start = v_start + temp_qn * (N - 1); 
            yaw_start = omega_start + temp_qn * N;
            gimble_vel_start = yaw_start + temp_qn * (N - 1); 
            timer.stop();
            if (!Solve(group[i][0], soft_sector_cons, refined_index_set)) {
                return false;
            }
        }
        // for (int i = 0; i < group.size(); i++) {
        // group_num = i;
        // int temp_qn = group[i].size();
        // x_start = 0;
        // y_start = x_start + temp_qn * N;
        // theta_start = y_start + temp_qn * N;
        // v_start = theta_start + temp_qn * N;
        // omega_start = v_start + temp_qn * (N - 1); 
        // yaw_start = omega_start + temp_qn * N;
        // gimble_vel_start = yaw_start + temp_qn * (N - 1); 
        // timer.stop();
        // if (!Solve(group[i][0], soft_sector_cons, refined_index_set) || timer.elapsedSeconds() > 60) {
        // return false;
        // }
        // }
        // return true;
        UpdateCost(plan, robot_cost_set, -1);
        // while (!costMap.empty()) {
        int asd;
        while (UpdateConflictPair(plan, relative_pair_updated, 0, asd)) {
            // int current_ind = costMap.back().index;
            // int current_ind_conflict;
            // if (!UpdateConflictPair(plan, relative_pair_updated, current_ind, current_ind_conflict)) {
            // costMap.pop_back();
            // ROS_WARN("No conflict found for robot%d!", current_ind);
            // continue;
            // }
            // group.clear();
            // group.push_back({current_ind}); 
            // group.push_back({current_ind_conflict}); 
            for (int i = 0; i < group.size(); i++) {
                int temp_qn = group[i].size();
                x_start = 0;
                y_start = x_start + temp_qn * N;
                theta_start = y_start + temp_qn * N;
                v_start = theta_start + temp_qn * N;
                omega_start = v_start + temp_qn * (N - 1);
                yaw_start = omega_start + temp_qn * N;
                gimble_vel_start = yaw_start + temp_qn * (N - 1); 
                group_num = i; // 组的序列号
                group_qn = temp_qn; // 当前组的机器人个数
                vertice_start = (N * 3 + (N - 1) * 2) * group_qn;

                // selected_pair_new.clear(); // 用于存放同一组相同优先级机器人之间的所有碰撞
                selected_pair_old.clear(); // 用于存放当前组中机器人与其他机器人之间的所有碰撞
                // std::set<int> new_set; // 存放当前组

                // for(int j = 0; j < group_qn; j++)
                // new_set.insert(group[0][j]); 

                int n1, n2;
                for (int j = 0; j < relative_pair_updated.size(); j++){ // j可以表示为time step
                    for (int k = 0; k < relative_pair_updated[j].size(); k++){
                        n1 = relative_pair_updated[j][k].at(0);
                        n2 = relative_pair_updated[j][k].at(1);

                        if (n1 == group[i][0] && std::find(robot_constraints[n1].begin(), robot_constraints[n1].end(), n2) != robot_constraints[n1].end())
                        {
                            selected_pair_old.emplace_back(std::array<int, 3>{j, n1, n2});
                        }
                        else if (n2 == group[i][0] && std::find(robot_constraints[n2].begin(), robot_constraints[n2].end(), n1) != robot_constraints[n2].end())
                        {
                            selected_pair_old.emplace_back(std::array<int, 3>{j, n2, n1});
                        }
                        // else if (new_set.count(n1) == 1 && new_set.count(n2) == 1)
                        // {
                        // selected_pair_new.emplace_back(std::array<int, 3>{j, n1, n2});
                        // }
                    }
                }

                selected_pair_old.clear();
                int nu = group[group_num][0];
                for (int i = 0; i < 3; i++) {
                    if (i != nu) {
                        for (int j = 0; j < N; j++) {
                            selected_pair_old.push_back({j, nu, i});
                        }
                    }
                }

                bool group_ok =Solve(i, soft_sector_cons, refined_index_set);
                // timer.stop();
                // if (timer.elapsedSeconds() > 60) {
                // return false;
                // }
                if (group_ok) {
                    ROS_INFO_STREAM("Solve robot" << group[i][0] << " success!");
                }
                else {
                    return false;
                }
                UpdateCost(plan, robot_cost_set, group[i][0]);
                // 如果规划失败，则将失败的组插入到最高优先级，重新进行规划
                // if ((!group_ok) && (!param.random_group))
                // {
                // std::vector <int> group_temp = group[i];
                // group.erase(group.begin() + i);
                // group.insert(group.begin(), group_temp);
                // i = -1;
                // old_set.clear();
                // ROS_INFO_STREAM("Try again");
                // continue;
                // }
                // 优化完成后存放优化后了的组的机器人索引值
                // for(int j = 0; j < group_qn; j++)
                // old_set.insert(group[0][j]);

                ROS_INFO_STREAM("Optimization Success!");
                // ROS_INFO_STREAM("Cost=" << cost);
            }
            for (int i = 0; i < group.size(); i++) {
                int temp_qn = group[i].size();
                x_start = 0;
                y_start = x_start + temp_qn * N;
                theta_start = y_start + temp_qn * N;
                v_start = theta_start + temp_qn * N;
                omega_start = v_start + temp_qn * (N - 1);
                yaw_start = omega_start + temp_qn * N;
                gimble_vel_start = yaw_start + temp_qn * (N - 1); 
                group_num = i; // 组的序列号
                group_qn = temp_qn; // 当前组的机器人个数
                vertice_start = (N * 3 + (N - 1) * 2) * group_qn;

                // selected_pair_new.clear(); // 用于存放同一组相同优先级机器人之间的所有碰撞
                selected_pair_old.clear(); // 用于存放当前组中机器人与其他机器人之间的所有碰撞
                // std::set<int> new_set; // 存放当前组

                // for(int j = 0; j < group_qn; j++)
                // new_set.insert(group[0][j]); 

                int n1, n2;
                for (int j = 0; j < relative_pair_updated.size(); j++){ // j可以表示为time step
                    for (int k = 0; k < relative_pair_updated[j].size(); k++){
                        n1 = relative_pair_updated[j][k].at(0);
                        n2 = relative_pair_updated[j][k].at(1);

                        if (n1 == group[i][0] && std::find(robot_constraints[n1].begin(), robot_constraints[n1].end(), n2) != robot_constraints[n1].end())
                        {
                            selected_pair_old.emplace_back(std::array<int, 3>{j, n1, n2});
                        }
                        else if (n2 == group[i][0] && std::find(robot_constraints[n2].begin(), robot_constraints[n2].end(), n1) != robot_constraints[n2].end())
                        {
                            selected_pair_old.emplace_back(std::array<int, 3>{j, n2, n1});
                        }
                        // else if (new_set.count(n1) == 1 && new_set.count(n2) == 1)
                        // {
                        // selected_pair_new.emplace_back(std::array<int, 3>{j, n1, n2});
                        // }
                    }
                }

                selected_pair_old.clear();
                int nu = group[group_num][0];
                for (int i = 0; i < 3; i++) {
                    if (i != nu) {
                        for (int j = 0; j < N; j++) {
                            selected_pair_old.push_back({j, nu, i});
                        }
                    }
                }

                bool group_ok =Solve(i, soft_sector_cons, refined_index_set);
                // timer.stop();
                // if (timer.elapsedSeconds() > 60) {
                // return false;
                // }
                if (group_ok) {
                    ROS_INFO_STREAM("Solve robot" << group[i][0] << " success!");
                }
                else {
                    return false;
                }
                UpdateCost(plan, robot_cost_set, group[i][0]);
                // 如果规划失败，则将失败的组插入到最高优先级，重新进行规划
                // if ((!group_ok) && (!param.random_group))
                // {
                // std::vector <int> group_temp = group[i];
                // group.erase(group.begin() + i);
                // group.insert(group.begin(), group_temp);
                // i = -1;
                // old_set.clear();
                // ROS_INFO_STREAM("Try again");
                // continue;
                // }
                // 优化完成后存放优化后了的组的机器人索引值
                // for(int j = 0; j < group_qn; j++)
                // old_set.insert(group[0][j]);

                ROS_INFO_STREAM("Optimization Success!");
                // ROS_INFO_STREAM("Cost=" << cost);
            }
        }
        ROS_WARN("Formation generation success!");
        timer_step.stop();
        ROS_INFO_STREAM("MPC runtime: " << timer_step.elapsedSeconds());
        double cost_total = 0.0;
        for (int i = 0; i < robot_cost_set.size(); i++) {
            cost_total += robot_cost_set[i];
        }
        ROS_INFO_STREAM("Total cost: " << cost);
        for (int qi = 0; qi < qn; qi++)
        {
            nav_msgs::Path path;
            path.header.frame_id = "map";
            path.header.stamp = ros::Time::now();
            for (int i = 0; i < N; i++)
            {
                pp.pose.position.x = plan[qi][i][0];
                pp.pose.position.y = plan[qi][i][1];
                pp.pose.orientation = tf::createQuaternionMsgFromYaw(plan[qi][i][2]);
                path.poses.push_back(pp);
                // 确定路径长度
                if (fabs(plan[qi][i][0] - plan[qi][N-1][0]) < 0.01){
                    if(fabs(plan[qi][i][1] - plan[qi][N-1][1]) < 0.01){
                        break;
                    }
                }
            }
            // way_point_pub[qi].publish(path);
        }
        plan_set = plan;
        return true;
    }

    private:
    std::vector<std::vector<double>> human_traj;
    std::vector<std::vector<std::vector<std::vector<double>>>> constraint_set;
    std::vector<double> time_cons;
    std::vector<std::pair<int, double>> st_time_cons;
    std::vector<std::vector<std::vector<std::vector<double>>>> view_region_hyper_set;
    std::vector<std::vector<std::vector<double>>> opt_theta_set;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::vector<std::vector<Eigen::Vector3d>> ref_paths;
    SwarmPlanning::Param param;
    std::vector<double>& theta_set;
    std::vector<std::vector<int>>& soft_sector_cons;
    std::vector<std::vector<Eigen::Vector2d>>& mr_goal_set;

    initTraj_t initTraj;
    std::vector<double> T;
    // SFC_t SFC;

    int M, phi, outdim;

    std::vector<Eigen::MatrixXd> coef;
};
