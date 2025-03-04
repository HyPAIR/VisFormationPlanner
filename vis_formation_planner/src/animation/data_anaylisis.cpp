/**
 * file complete_animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief complete animation 
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <iostream>
#include <ros/ros.h>
#include <to_solver/param.hpp>
#include <to_solver/mission.hpp>
#include "vis_formation_planner/vis_formation_planner.h"
#include "vis_formation_planner/math/generate_obs.h"
#include "vis_formation_planner/visualization/plot.h"
#include "vis_formation_planner/yaml_all.h"
#include "vis_formation_planner/visualization/matplotlibcpp.h"
#include <nav_msgs/Path.h>
#include "vis_formation_planner/visualization/visualization.h"
#include "vis_formation_planner/utils.h"
#include <vector>
#include <algorithm>
#include <numeric> 

using namespace vis_formation_planner;
using namespace std;
namespace plt = matplotlibcpp;

void FindMinMaxAvg(const std::vector<double>& data, double& min_val, double& max_val, double& avg_val) {
    if (data.empty()) {
        std::cerr << "Error: The vector is empty!" << std::endl;
        min_val = max_val = avg_val = 0.0;
        return;
    }

    min_val = *std::min_element(data.begin(), data.end());
    max_val = *std::max_element(data.begin(), data.end());
    avg_val = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

void ComputeMeanAndVariance(const std::vector<double>& data, double& mean, double& variance) {
    if (data.empty()) {
        std::cerr << "Error: The vector is empty!" << std::endl;
        mean = variance = 0.0;
        return;
    }

    mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    double sum_sq_diff = 0.0;
    for (const auto& value : data) {
        sum_sq_diff += (value - mean) * (value - mean);
    }
    variance = sum_sq_diff / data.size();  
}

int main(int argc, char* argv[])
{
    std::shared_ptr<visualize::visualize> visPtr_;
    auto config_ = std::make_shared<PlannerConfig>();
    config_->vehicle.InitializeDiscs();
    auto env = std::make_shared<Environment>(config_);
    std::string tail = "0";
    int j = 0;
	math::GenerateObstacle generateobs;
    std::vector<math::Polygon2d> polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    bool plot_traj = false;

    double x, y;    
    double alpha = 100 * M_PI / 180;  
    double L = 5.0;       
    std::vector<std::vector<double>> height_set;
    std::vector<double> height;

    std::vector<int> num_obs = {30, 50, 70, 90, 110};
    std::vector<int> num_robots = {3, 4};
    std::vector<std::string> num_benchmark = {"ours/", "ho/", "wang/", "zhou/"};
    for (int m = 0; m < num_robots.size(); m++) {
        for (int k = 0; k < num_obs.size(); k++) {
            for (int i = 0; i < num_benchmark.size(); i++) {
                std::vector<std::vector<double>> data;
                int success_cases = 0;
                for (int j = 0; j < 15; j++) {
                    std::string obs_path = "/home/weijian/VisFormation/src/vis_formation_planner/result/obstacle" + std::to_string(num_obs[k]) + std::to_string(j) + ".yaml";
                    if(!ReadRandomObstacle(num_obs[k], height_set, obs_path, height, polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe, poly_vertices_set, generateobs)){
                        continue;
                    }
                    std::vector<std::vector<std::vector<double>>> plan;
                    if (!generate_traj_fg_("/home/weijian/VisFormation/src/vis_formation_planner/result/" + num_benchmark[i] + std::to_string(num_robots[m]) + "/_fg_"  + std::to_string(num_obs[k]) + std::to_string(j) + ".yaml", num_robots[m], plan)) {
                        continue;
                    }
                    success_cases++;
                    std::vector<std::vector<double>> human_traj = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/" + num_benchmark[i] + std::to_string(num_robots[m]) + "/" + "_human_traj_" + std::to_string(num_obs[k]) + std::to_string(j) +".yaml");
                    env->polygons() = polys;
                    std::vector<double> theta_set_;
                    std::vector<double> theta_set;
                    std::vector<int> index_set;
                    std::vector<double> visibility_ratio, traj_length, acceleration, omega_vel;
                    double traj_length_a, traj_length_s;
                    double acceleration_a, acceleration_s, omega_velocity_a, omega_velocity_s;
                    for (int n = 0; n < num_robots[m]; n++) {
                        for (int o = 0; o < plan[n].size(); o++) {
                        }
                        double len = 0.0;
                        for (int o = 1; o < plan[n].size(); o++) {
                            double acc = (plan[n][o][3] - plan[n][o - 1][3]) / 0.2;
                            double omg = (plan[n][o][4] - plan[n][o - 1][4]) / 0.2;
                            acceleration.push_back(fabs(acc));
                            omega_vel.push_back(fabs(omg));
                            len += hypot(plan[n][o][1] - plan[n][o - 1][1], plan[n][o][0] - plan[n][o - 1][0]);
                        }
                        traj_length.push_back(len);
                        std::vector<std::vector<std::vector<double>>> fov_set(plan.size()), fov_invalid(plan.size());
                        for (int o = 0; o < plan[n].size(); o++) {
                            double x_vec = human_traj[o][0] - plan[n][o][0];
                            double y_vec = human_traj[o][1] - plan[n][o][1];
                            double theta = M_PI / 2 - plan[n][o][5];
                            double x_vec_ = x_vec * cos(theta) - y_vec * sin(theta);
                            double y_vec_ = x_vec * sin(theta) + y_vec * cos(theta);
                            Eigen::Vector2d p = {plan[n][o][0], plan[n][o][1]};
                            Eigen::Vector2d center = {human_traj[o][0], human_traj[o][1]};
                            if (env->checkRayValid(p, center) && atan2(y_vec_, x_vec_) < 5 * M_PI / 6 && atan2(y_vec_, x_vec_) > M_PI / 6) {
                                fov_set[n].push_back({x_vec_, y_vec_});
                            }
                            else {
                                fov_invalid[n].push_back({x_vec_, y_vec_});
                            }
                        }
                        std::vector<double> x1, y1;
                        std::vector<double> x3, y3;
                        for (int i = 0; i < fov_set[n].size(); i++) {
                            x3.push_back(fov_set[n][i][0]);
                            y3.push_back(fov_set[n][i][1]);
                        }
                        std::vector<double> x4, y4;
                        for (int i = 0; i < fov_invalid[n].size(); i++) {
                            x4.push_back(fov_invalid[n][i][0]);
                            y4.push_back(fov_invalid[n][i][1]);
                        }
                        // std::cout << "Visibility Success rate: " << double(x3.size()) / double(plan[0].size()) * 100 << "%" << std::endl;
                        double vis_rate = double(x3.size()) / double(plan[0].size()) * 100;
                        visibility_ratio.push_back(vis_rate);
                    }
                    double success_rate = double(success_cases) / 15.0;
                    ComputeMeanAndVariance(traj_length, traj_length_a, traj_length_s);
                    ComputeMeanAndVariance(acceleration, acceleration_a, acceleration_s);
                    ComputeMeanAndVariance(omega_vel, omega_velocity_a, omega_velocity_s);
                    double min_val, max_val, avg_val;
                    FindMinMaxAvg(visibility_ratio, min_val, max_val, avg_val);
                    data.push_back({success_rate, min_val, max_val, avg_val, traj_length_a, traj_length_s, acceleration_a, acceleration_s, omega_velocity_a, omega_velocity_s});                
                }
                writeVectorsToYaml(data, "/home/weijian/VisFormation/src/vis_formation_planner/result/data_anaylisi/" + std::to_string(num_robots[m]) + "/" + num_benchmark[i] + "_result" + std::to_string(num_obs[k]) + ".yaml");
            }
        }
    }
	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
