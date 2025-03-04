#pragma once

#include "init_traj_planner.hpp"
#include <environment.hpp>
#include "vis_formation_planner/math/pose.h"
#include "vis_formation_planner/math/vec2d.h"
#include "vis_formation_planner/math/generate_obs.h"
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>

using namespace vis_formation_planner;

using namespace libMultiRobotPlanning;

class ECBSPlanner : public InitTrajPlanner{
public:
    ECBSPlanner(std::vector<math::Polygon2d>  _polys,
                std::vector<std::vector<std::vector<vis_formation_planner::visualization::Vector>>> _hPolys_ht_sets,
                std::vector<std::vector<double>> _start_states,
                std::vector<std::vector<double>> _goal_states,
                SwarmPlanning::Param _param,
                bool _homotopy_cons)
                : InitTrajPlanner(std::move(_polys),
                                  std::move(_hPolys_ht_sets),
                                  std::move(_start_states),
                                  std::move(_goal_states),
                                  std::move(_param),
                                  std::move(_homotopy_cons))
    {
        setObstacles();
    }

    bool update(bool log) override {
        if (!setWaypoints()) {
            return false;
        }
        std::vector<double> quad_size(start_states.size(), 0.2);
        Environment_fg mapf(dimx, dimy, dimz, ecbs_obstacles, ecbs_goalLocations, quad_size, param.grid_xy_res);
        ECBS<State, Action, int, Conflict, libMultiRobotPlanning::Constraints, Environment_fg> ecbs(mapf, param.ecbs_w);
        std::vector<PlanResult<State, Action, int>> solution;

        // Execute ECBS algorithm
        bool success = ecbs.search(ecbs_startStates, solution, param.log);
        if(!success){
            ROS_ERROR("ECBSPlanner: ECBS Failed!");
            return false;
        }

        // Update segment time
        int cost = 0;
        int makespan = 0;
        for (const auto &s : solution) {
            cost += s.cost;
            //makespan = std::max<int>(makespan, s.cost);
            makespan = std::max<int>(makespan, s.states.size());
        }
        for(int i = 0; i <= makespan + 2; i++){
            T.emplace_back(i * param.time_step);
        }
        if(log) {
            ROS_INFO_STREAM("ECBSPlanner: M=" << T.size() - 1 << ", makespan=" << T.back());
        }

        // Append start, goal points to both ends respectively
        initTraj.resize(solution.size());
        auto [g_x_min, g_x_max, g_y_min, g_y_max] = findStateBounds(start_states, goal_states, 2.0);
        for (size_t a = 0; a < solution.size(); ++a) {
            initTraj[a].emplace_back(octomap::point3d(start_states[a][0],
                                                      start_states[a][1],
                                                      start_states[a][2]));

            for (const auto &state : solution[a].states) {
                initTraj[a].emplace_back(octomap::point3d(state.first.x * param.grid_xy_res + g_x_min,
                                                          state.first.y * param.grid_xy_res + g_y_min,
                                                          state.first.z));
            }
            while(initTraj[a].size() <= makespan + 2){
                initTraj[a].emplace_back(octomap::point3d(goal_states[a][0],
                                                          goal_states[a][1],
                                                          goal_states[a][2]));
            }
        }
        return true;
    }

    bool isInsidePolygon(const std::vector<math::Polygon2d>& polygons, const math::Pose& pose) {
        bool inside = false;
        for (auto& polygon : polygons)
            for (int i = 0, j = polygon.points().size() - 1; i < polygon.points().size(); j = i++) {
                if (((polygon.points()[i].y() > pose.y()) != (polygon.points()[j].y() > pose.y())) &&
                    (pose.x() < (polygon.points()[j].x() - polygon.points()[i].x()) * (pose.y() - polygon.points()[i].y()) / 
                    (polygon.points()[j].y() - polygon.points()[i].y()) + polygon.points()[i].x())) {
                    inside = !inside;
                }
            }

        return inside;
    }

    bool isInsideCorridor(const std::vector<vis_formation_planner::visualization::Vector>& hPolys, const math::Pose& pose) {
        for (int i = 0; i < hPolys.size(); i++) {
            if (pose.x() * hPolys[i][0] + pose.y() * hPolys[i][1] - hPolys[i][2] >= 0) {
                return false;
            }
        }
        return true;
    }

private:
    std::unordered_set<Location> ecbs_obstacles;
    std::vector<State> ecbs_startStates;
    std::vector<Location> ecbs_goalLocations;

    // Find the location of obstacles in grid-space
    bool setObstacles(){
        auto [g_x_min, g_x_max, g_y_min, g_y_max] = findStateBounds(start_states, goal_states, 2.0);
        Timer timer_step;
        timer_step.reset();
        int x,y,z;
        for (double k = grid_z_min; k < grid_z_max + SP_EPSILON; k += param.grid_z_res ){
            for (double i = g_x_min; i < g_x_max + SP_EPSILON; i += param.grid_xy_res*0.2 ){
                for (double j = g_y_min; j < g_y_max + SP_EPSILON; j += param.grid_xy_res*0.2 ){
                    // octomap::point3d cur_point(i,j,k);
                    math::Pose cur_point(i, j, k);
                    x = (int)round((i - g_x_min) / (param.grid_xy_res*0.2));
                    y = (int)round((j - g_y_min) / (param.grid_xy_res*0.2));
                    z = (int)round((k - grid_z_min) / param.grid_z_res);
                    // float dist = distmap_obj.get()->getDistance(cur_point);
                    // if(dist < 0){
                    //     return false;
                    // }

                    // To prevent obstacles from putting between grid points, grid_margin is used
                    // if (dist < r + param.grid_margin){
                    if (isInsidePolygon(polys, cur_point)) {
                        ecbs_obstacles.insert(Location(x, y, z));
                    }
                    else {
                        if (homotopy_cons) {
                            bool isHC = false;
                            for (int i_ = 0; i_ < hPolys_ht_sets.size(); i_++) {
                                for (int j_ = 0; j_ < hPolys_ht_sets[i_].size(); j_++) {
                                    if (isInsideCorridor(hPolys_ht_sets[i_][j_], cur_point)) {
                                        isHC = true;
                                        goto end_loops; 
                                    }
                                }
                            }
                            end_loops:
                            if (!isHC) {
                                ecbs_obstacles.insert(Location(x, y, z));
                            }
                        }
                    }
                }
            }
        }
        timer_step.stop();
        // ROS_INFO_STREAM("setObstacles runtime: " << timer_step.elapsedSeconds());
        return true;
    }

    // Set start, goal points of ECBS
    bool setWaypoints(){
        auto [g_x_min, g_x_max, g_y_min, g_y_max] = findStateBounds(start_states, goal_states, 2.0);
        Timer timer_step;
        timer_step.reset();
        int xig, yig, zig, xfg, yfg, zfg;
        for(int i = 0; i < start_states.size(); i++){
            // For start, goal point of ECBS, we use the nearest grid point.
            xig = (int)round((start_states[i][0] - g_x_min) / param.grid_xy_res);
            yig = (int)round((start_states[i][1] - g_y_min) / param.grid_xy_res);
            zig = (int)round((start_states[i][2] - grid_z_min) / param.grid_z_res);
            xfg = (int)round((goal_states[i][0] - g_x_min) / param.grid_xy_res);
            yfg = (int)round((goal_states[i][1] - g_y_min) / param.grid_xy_res);
            zfg = (int)round((goal_states[i][2] - grid_z_min) / param.grid_z_res);

            if(ecbs_obstacles.find(Location(xig * 5 , yig * 5 , zig)) != ecbs_obstacles.end()){
                ROS_ERROR_STREAM("ECBSPlanner: start of agent " << i << " is occluded by obstacle");
                return false;
            }
            if(ecbs_obstacles.find(Location(xfg * 5, yfg * 5 , zfg)) != ecbs_obstacles.end()) {
                ROS_ERROR_STREAM("ECBSLauncher: goal of agent " << i << " is occluded by obstacle");
                return false;
            }

            if (param.initial_angle)
            {
                ecbs_startStates.emplace_back(State(0, xig, yig, start_states[i][2]));
                ecbs_goalLocations.emplace_back(Location(xfg, yfg, goal_states[i][2]));
            }
            else
            {
                ecbs_startStates.emplace_back(State(0, xig, yig, zig));
                ecbs_goalLocations.emplace_back(Location(xfg, yfg, zfg));
            }
        }   
        timer_step.stop();
        // ROS_INFO_STREAM("setWaypoints runtime: " << timer_step.elapsedSeconds());
        return true;
    }
    std::tuple<double, double, double, double> findStateBounds(
        const std::vector<std::vector<double>>& start_states,
        const std::vector<std::vector<double>>& goal_states,
        double margin) {
        
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::lowest();

        auto updateBounds = [&](const std::vector<std::vector<double>>& states) {
            for (const auto& state : states) {
                if (state.size() < 2) continue; 
                x_min = std::min(x_min, state[0]);
                x_max = std::max(x_max, state[0]);
                y_min = std::min(y_min, state[1]);
                y_max = std::max(y_max, state[1]);
            }
        };

        updateBounds(start_states);
        updateBounds(goal_states);

        x_min -= margin;
        x_max += margin;
        y_min -= margin;
        y_max += margin;

        return {x_min, x_max, y_min, y_max};
    }
};
