#pragma once

#include <to_solver/sp_const.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <to_solver/param.hpp>
#include <to_solver/mission.hpp>
#include "vis_formation_planner/math/polygon2d.h"

using namespace vis_formation_planner;

class InitTrajPlanner {
public:
    initTraj_t initTraj; // discrete initial trajectory: pi_0,...,pi_M
    std::vector<double> T; // segment time: T_0,...,T_M

    virtual bool update(bool log) = 0;
    InitTrajPlanner() {}
    InitTrajPlanner(std::vector<math::Polygon2d>  _polys,
                    std::vector<std::vector<std::vector<vis_formation_planner::visualization::Vector>>> _hPolys_ht_sets,
                    std::vector<std::vector<double>> _start_states,
                    std::vector<std::vector<double>> _goal_states,
                    SwarmPlanning::Param _param,
                    bool _homotopy_cons)
            : polys(std::move(_polys)),
              hPolys_ht_sets(std::move(_hPolys_ht_sets)),
              start_states(std::move(_start_states)),
              goal_states(std::move(_goal_states)),
              param(std::move(_param)),
              homotopy_cons(std::move(_homotopy_cons))
    {
        grid_x_min = ceil((param.world_x_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_y_min = ceil((param.world_y_min + SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_z_min = ceil((param.world_z_min + SP_EPSILON) / param.grid_z_res) * param.grid_z_res;

        grid_x_max = floor((param.world_x_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_y_max = floor((param.world_y_max - SP_EPSILON) / param.grid_xy_res) * param.grid_xy_res;
        grid_z_max = floor((param.world_z_max - SP_EPSILON) / param.grid_z_res) * param.grid_z_res;

        dimx = (int)round((grid_x_max - grid_x_min) / param.grid_xy_res) + 1;
        dimy = (int)round((grid_y_max - grid_y_min) / param.grid_xy_res) + 1;
        dimz = (int)round((grid_z_max - grid_z_min) / param.grid_z_res) + 1;
    }

    SwarmPlanning::Mission mission;
protected:
    std::vector<math::Polygon2d> polys;
    std::vector<std::vector<std::vector<vis_formation_planner::visualization::Vector>>> hPolys_ht_sets;
    SwarmPlanning::Param param;
    bool homotopy_cons;
    std::vector<std::vector<double>> start_states;
    std::vector<std::vector<double>> goal_states;
    double grid_x_min, grid_y_min, grid_z_min, grid_x_max, grid_y_max, grid_z_max;
    int dimx, dimy, dimz;
};