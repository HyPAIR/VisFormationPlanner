// Created by weijian on 17/11/23.
#pragma once
#include "vehicle_model.h"

namespace vis_formation_planner {

struct PlannerConfig {
   double xy_resolution = 0.5;
   double theta_resolution = 0.1;
   double step_size = 0.05;
   int next_node_num = 6;
   double grid_xy_resolution = 1.0;
   double forward_penalty = 0.5;
   double backward_penalty = 1.0;
   double gear_change_penalty = 5.0;
   double steering_penalty = 0.5;
   double steering_change_penalty = 1.0;
   double d_lb = 1.5;
   double d_ub = 5;
  /**
   * Minimum number of finite elements used to discretize an OCP
   */
  int min_nfe = 20;

  /**
   *
   */
  double time_step = 0.1;

  /**
   * maximum iteration count for corridor expansion
   */
  int corridor_max_iter = 1000;

  /**
   * increment limit for corridor expansion
   */
  double corridor_incremental_limit = 20.0;

  /**
   * Weighting parameter for control input acceleration
   */
  double opti_w_phi = 1.0;
  double opti_w_a = 1.0;
  double opti_t = 1.0;
  double factor_a = 0.9;
  double factor_b = 1.1;
  
  /**
   * weighting parameter for control input omega
   */
  double opti_w_omega = 1.0;
  
  /**
   * weighting parameter for control input for diff-drive robot
   */
  double opti_w_diff_drive = 0.05;

  double opti_w_x = 1.0;
  double opti_w_y = 1.0;
  double opti_w_err = 1.0;
  double opti_w_theta = 1.0;

  int opti_inner_iter_max = 100;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e4;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-4;

  VehicleModel vehicle;
  FormationConfig formation;
};

}