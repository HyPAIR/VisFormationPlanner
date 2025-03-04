/**
 * file fk_test.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief animation
 * data 2024-06-14
 * 
 * @copyright Copyroght(c) 2024
*/
#include <iostream>
#include <ros/ros.h>
#include "vis_formation_planner/vis_formation_planner.h"
#include "vis_formation_planner/math/generate_obs.h"
#include "vis_formation_planner/visualization/plot.h"
#include "vis_formation_planner/yaml_all.h"
#include "traj_tracking/matplotlibcpp.h"
#include "vis_formation_planner/forward_kinematics.h"
#include <nav_msgs/Path.h>
#include <random>
#include "vis_formation_planner/visualization/visualization.h"
#include "vis_formation_planner/robot_angle_optimizaer.h"
#include "vis_formation_planner/utils.h"

// Parameters
#include <to_solver/param.hpp>
#include <to_solver/mission.hpp>
#include <to_solver/timer.hpp>

// Submodules
#include <to_solver/ecbs_planner.hpp>
#include <to_solver/corridor.hpp>
#include <to_solver/prioritized_traj_optimization.hpp>
#include <to_solver/prioritized_traj_publisher.hpp>

#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace vis_formation_planner;
namespace plt = matplotlibcpp;

int main(int argc, char* argv[])
{
    std::shared_ptr<visualize::visualize> visPtr_;
    ros::init(argc, argv, "ring_visualizer");
    ros::NodeHandle nh;
    visualization::Init(nh, "odom", "/liom_test_vis");

    ros::Publisher hPolyPub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedra", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(10);
    std::vector<int> num_obs = {30};
    int num_robot = 3;
    for (int obs_num_ind = 0; obs_num_ind < num_obs.size(); obs_num_ind++) {
      int index = 0;
    my_label:
      while (index < 100) {
    std::vector<double> solving_time;
    std::vector<std::vector<double>> height_set;
    std::vector<std::vector<double>> obstacle;
    std::vector<double> height;
    std::vector<math::Polygon2d> polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe;
    std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    math::GenerateObstacle generateobs;
    GenerateRandomObstacle(num_obs[obs_num_ind], height_set, obstacle, height, polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe, poly_vertices_set, generateobs);
    auto config_ = std::make_shared<PlannerConfig>();
    config_->vehicle.InitializeDiscs();
    auto env = std::make_shared<Environment>(config_);
    auto planner_ = std::make_shared<LiomLocalPlanner>(config_, env);
    env->polygons() = polys_inflat_ecbs;
    // Test parameters
    Eigen::Vector2d center, center_prev; // Center of the environment
    std::vector<double> initial_angles = {-5 * M_PI / 6, -M_PI / 6, M_PI / 2};  // Initial angles
    std::vector<double> initial_angles_ = {7 * M_PI / 6, 11 * M_PI / 6, M_PI / 2};  // Initial angles - [0, 2pi]
    // std::vector<double> initial_angles = {-M_PI, -M_PI / 2, 0, M_PI / 2};  // Initial angles
    // std::vector<double> initial_angles_ = {M_PI, 3 * M_PI / 2, 2 * M_PI, M_PI / 2};  // Initial angles - [0, 2pi]
    double theta_neighbor = M_PI / 3; // Initial neighbor range (22.5 degrees)
    double min_distance = 2.0;       // Minimum visible distance
    double max_distance = 5.0;       // Maximum visible distance
    double theta_s = M_PI / 4;       // Minimum cumulative visible angle (45 degrees)

    std::vector<std::pair<double, double>> visible_regions;
    std::vector<std::vector<std::pair<double, double>>> visible_regions_set;
    // Eigen::Vector2d p = {2.527, 4.293};
    // Eigen::Vector2d c = {1.524, 2.601};
    // if(!env->checkRayValid(p, c)) {
    //   std::cout << "Invalid Ray!" << std::endl;
    // }
    // std::cout << "Visible Regions:" << std::endl;
    // for (const auto& region : visible_regions) {
    //     std::cout << "Left: " << region.first << ", Right: " << region.second << std::endl;
    // }

    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> pointsHyperplane;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 2>> pointsVertices;
    Timer timer_step, timer_step_ig, timer_step_to;
    // ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(initial_angles.size());
    Timer timer_total;
    double start_time, current_time;
    int count = 0;

    while (ros::ok()) {
        if (index == 100) {
          break;
        }
        math::Pose start = {0.0, 0.0, 0.0};
        math::Pose goal = {50.0, 50.0, 0.0};
        double x_c = 0.0;        // Circle center x
        double y_c = 0.0;        // Circle center y
        int num_points = 50;     // Number of points for the arc
        std::vector<std::vector<double>> theta_set;
        std::vector<std::vector<Eigen::Vector2d>> intervals_set;
        std::vector<std::vector<double>> occ_set(num_robot);
        std::vector<std::vector<std::vector<Eigen::Vector2d>>> polygons_set;
        std::vector<std::vector<Eigen::MatrixXd>> view_region_sets(num_robot);
        std::vector<std::vector<double>> dis_scaler_set;
        std::vector<Eigen::Vector2d> sampled_human_traj;
        std::vector<std::vector<Eigen::Vector2d> > mr_goal_set;
        std::vector<std::vector<Eigen::Vector3d>> ref_paths(num_robot);
        std::vector<std::vector<Eigen::Vector3d>> ref_paths_2d(num_robot);
        std::vector<std::vector<std::vector<std::vector<double>>>> view_region_hyper_set(num_robot);
        // std::vector<Eigen::Vector2d> points;
        // for (int i = 0; i < num_robot; i++) {
        //   points.push_back({start.x() + (min_distance + max_distance) / 2 * cos(initial_angles[i]), start.y() + (min_distance + max_distance) / 2 * sin(initial_angles[i])});
        // }
        // mr_goal_set.push_back({points});
        /*------------------- human motion prediction -------------------*/ 
        CoarsePathPlanner predicted_human_path(config_, env);
        std::vector<math::Pose> human_traj;
        std::vector<std::vector<double>> human_traj_;
        std::vector<vis_formation_planner::visualization::Vector> corridor_sets;
        if(!predicted_human_path.Plan(start, goal, human_traj, corridor_sets)) {
          ROS_ERROR("NO PREDICTED TRAJECTORY FOUND!");
          index++;
          if (index == 100) {
            break;
          }
          goto my_label;
        }
        std::vector<double> xs, ys;
        for(auto &pose: human_traj) {
          xs.push_back(pose.x()); ys.push_back(pose.y());
        }
        visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, 1, "Coarse Path");
        visualization::Trigger();
        env->polygons() = polys_inflat_ecbs;
        for (int i = 0; i < poly_vertices_set.size(); i++) {
            visPtr_->publishFilledQuadrilateral(marker_pub, 
                                       poly_vertices_set[i][0].x(), poly_vertices_set[i][0].y(), 
                                       poly_vertices_set[i][1].x(), poly_vertices_set[i][1].y(), 
                                       poly_vertices_set[i][2].x(), poly_vertices_set[i][2].y(),
                                       poly_vertices_set[i][3].x(), poly_vertices_set[i][3].y(),
                                       i,
                                       "obstacle");
        }
        timer_step_ig.reset();
        auto human_traj_index = findIndices(human_traj);
        std::vector<Eigen::Vector2d> points_prev;
        // std::vector<double> fixed_angle_set = {M_PI, 3 * M_PI / 2, 2 * M_PI, M_PI / 2};
        std::vector<double> fixed_angle_set = {7 * M_PI / 6, 11 * M_PI / 6, M_PI / 2};
        double init_formation_angle = 0.0;
        for (int i = 0; i < human_traj_index.size(); i++) {
          /*------------------- compute visible annulus sectors -------------------*/ 
          center = {human_traj[human_traj_index[i]].x(), human_traj[human_traj_index[i]].y()};
          center_prev = {human_traj[human_traj_index[(i-1)==0?0:(i-1)]].x(), human_traj[human_traj_index[(i-1)==0?0:(i-1)]].y()};
          sampled_human_traj.push_back(center);
          std::vector<double> dis_scaler(initial_angles.size(), 1.0);
          bool recalculate = true;
          bool dis_scaler_updated = false;
          while (recalculate) {
            recalculate = false;
            visible_regions.clear();
            visible_regions_set.clear();
            // theta_s = 0.0;
            // theta_neighbor = M_PI / 2;
            for (int j = 0; j < initial_angles.size(); j++) {
              if (!env->find_visible_regions(dis_scaler_updated, j, center, initial_angles[j], theta_neighbor, min_distance, max_distance, dis_scaler, theta_s, visible_regions, points_prev)) {
                ROS_ERROR("NO VISIBLE REGIONS FOUND!");
                index++;
                if (index == 100) {
                  goto my_label;
                }
                goto my_label;
              }
              if (visible_regions.empty()) {
                index++;
                if (index == 100) {
                  goto my_label;
                }
                  goto my_label;
              }
              visible_regions_set.push_back(visible_regions);
            }

            /*------------------- Solve MIQP -------------------*/ 
            auto robotIntervals = normalizeAngleRanges(visible_regions_set);
            AngleOptimization optimizer(robotIntervals, initial_angles_, min_distance, max_distance, dis_scaler, center, points_prev);
            optimizer.optimize();

            std::vector<double> theta = optimizer.getTheta();
            theta_set.push_back(theta);
            for(int ind = 0; ind < initial_angles.size(); ind++) {
              initial_angles[ind] = theta[ind];
            }
            std::vector<int> selectedIntervals = optimizer.getSelectedIntervals();
            double diversity = optimizer.getDiversity();
            std::vector<Eigen::Vector2d> points;
            double t_l, t_r;
            std::vector<Eigen::Vector2d> intervals;
            for (size_t i = 0; i < theta.size(); ++i) {
                const auto& interval = robotIntervals[i][selectedIntervals[i]];
                std::cout << "Robot " << i << ": " << theta[i]
                          << " (Interval: [" << interval.first << ", " << interval.second << "])" << std::endl;
              auto point = calculateEndpoint(center, (min_distance + max_distance) * dis_scaler[i] / 2, theta[i]);
              Eigen::Vector2d visible_p;
              double theta_c;
              env->find_visible_neighbors_(i, center, point, min_distance * dis_scaler[i], max_distance * dis_scaler[i], visible_p, theta_c, t_l, t_r, points_prev);
              point = calculateEndpoint(center, (min_distance + max_distance) * dis_scaler[i] / 2, (t_l + t_r) / 2);
              points.push_back(point);
              intervals.push_back({t_l, t_r});
              occ_set[i].push_back((t_l + t_r) / 2);
            }
            points_prev = points;
            for (int ind = 0; ind < intervals.size(); ind++) {
              int tune_index;
              if (!env->visible_regions_feasible(ind, dis_scaler, intervals, points, tune_index)) {
                dis_scaler[tune_index] *= 1.1;
                recalculate = true;
                dis_scaler_updated = true;
                break;
              }
            }
            if (!recalculate) {
              intervals_set.push_back(intervals);
              mr_goal_set.push_back(points);
              // std::cout << "Maximum diversity (minimum angle difference): " << diversity << std::endl;
              
              /*------------------- compute the visible regions -------------------*/
              std::vector<std::vector<Eigen::Vector2d>> polygons;      
              std::vector<Eigen::Vector2d> polygon;
              Eigen::MatrixXd view_region;
              for (size_t i = 0; i < intervals.size(); ++i) {
                  std::vector<std::vector<double>> view_region_hyper;
                  env->compute2DViewRegion(points[i], center, min_distance * dis_scaler[i], max_distance * dis_scaler[i], intervals[i](0), intervals[i](1), polygon, view_region, view_region_hyper);
                  polygons.push_back(polygon);
                  view_region_sets[i].push_back(view_region);
                  view_region_hyper_set[i].push_back(view_region_hyper);
              }
              polygons_set.push_back(polygons);
              dis_scaler_set.push_back(dis_scaler);
            }
            ROS_INFO_STREAM("MIQP solving time: " << timer_step.elapsedSeconds());
            int ind = polygons_set.size() - 1;
            if (polygons_set.empty()) {
              index++;
              if (index == 100) {
                goto my_label;
              }
                goto my_label;
            }
            visPtr_->publishPolygon(polygons_set[ind], "odom", "polygon_" + std::to_string(ind), ind, marker_pub);
            visPtr_->publishAnnulus(ind, intervals_set[ind], sampled_human_traj[ind](0), sampled_human_traj[ind](1), dis_scaler_set[ind], min_distance, max_distance, marker_pub);
            drawPoints(mr_goal_set[ind], "odom", "red_points" + std::to_string(ind), ind, marker_pub);
          }
        }

        /*------------------- Plan Initial Trajectory -------------------*/
        env->polygons() = polys_inflat_ecbs;
        // Submodules
        std::shared_ptr<DynamicEDTOctomap> distmap_obj;
        std::shared_ptr<Corridor> corridor_obj;
        std::shared_ptr<MPCPlanner> MPCPlanner_obj;
        std::vector<std::pair<int, double>> st_time_cons;
        st_time_cons.push_back({0, 0.0});
        ROS_INFO("Multi-robot Trajectory Planning");
        std::vector<std::vector<std::vector<math::Vec2d>>> homo_vertices_sets;
        for (int i = 0; i < mr_goal_set.size() - 1; i++) {
          std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
          std::vector<std::vector<double>> start_states;
          std::vector<std::vector<double>> goal_states;
          std::vector<double> mr_temp_set_s, mr_temp_set_g;
          std::vector<std::vector<std::vector<std::vector<double>>>> hPolys_ht_sets(num_robot);
          for (int j = 0; j < mr_goal_set[i].size(); j++) {
            start_states.push_back({mr_goal_set[i][j][0], mr_goal_set[i][j][1], 1.0});
            goal_states.push_back({mr_goal_set[i + 1][j][0], mr_goal_set[i + 1][j][1], 1.0});
            // set the collision-free homotopy path
            Eigen::Vector2d p_start = {mr_goal_set[i][j][0], mr_goal_set[i][j][1]};
            Eigen::Vector2d human_start = {human_traj[human_traj_index[i]].x(), human_traj[human_traj_index[i]].y()};
            Eigen::Vector2d p_end = {mr_goal_set[i+1][j][0], mr_goal_set[i+1][j][1]};
            Eigen::Vector2d human_end = {human_traj[human_traj_index[i+1]].x(), human_traj[human_traj_index[i+1]].y()};
            auto path_st = env->interpolatePoints(p_start, human_start, 1);
            // Sample human trajectory
            std::vector<Eigen::Vector2d> human_path;
            double human_num_pts = human_traj_index[i+1] - human_traj_index[i];
            int pts_step = human_num_pts / 3.0;
            for (int k = human_traj_index[i]; k < human_traj_index[i+1]; k += pts_step) {
              if (k >= pts_step) {
                break;
              }
              human_path.push_back({human_traj[k].x(), human_traj[k].y()});
            }
            human_path.push_back({human_traj[human_traj_index[i+1]].x(), human_traj[human_traj_index[i+1]].y()});
            auto path_end = env->interpolatePoints(human_end, p_end, 1);
            path_st.insert(path_st.end(), human_path.begin(), human_path.end());
            path_st.insert(path_st.end(), path_end.begin(), path_end.end());
            std::vector<std::vector<std::vector<double>>> hPolys;
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> keyPts;
            std::vector<std::vector<math::Vec2d>> corri_vertices_set;
            env->generateSFC(path_st, 2.0, hPolys, keyPts, corri_vertices_set);
            hPolys_ht_sets[j] = hPolys;
            homo_vertices_sets.push_back(corri_vertices_set);
          }
          // for (int i_ = 0; i_ < homo_vertices_sets.size(); i_++) {
          //     for (int j = 0; j < homo_vertices_sets[i_].size(); j++) {
          //     auto color_poly = visualization::Color::Cyan;
          //     color_poly.set_alpha(0.1);
          //     visualization::PlotPolygon(math::Polygon2d(homo_vertices_sets[i_][j]), 0.1, color_poly, 0, "Corridor" + std::to_string(i_) + std::to_string(j));
          //   }
          // }
          // visualization::Trigger();  
          initTrajPlanner_obj.reset(new ECBSPlanner(polys_inflat_ecbs, hPolys_ht_sets, start_states, goal_states, param, true));
          // initTrajPlanner_obj.reset(new ECBSPlanner(polys_inflat, mission, param));
          if (!initTrajPlanner_obj.get()->update(param.log)) {
            ROS_WARN("Fail to be homotopically consistent, try to relax such constraint!");
            // visPtr_->publishPolygon(polygons_set[i], "odom", "polygon_" + std::to_string(i), i, marker_pub);
            // visPtr_->publishAnnulus(i, intervals_set[i], sampled_human_traj[i](0), sampled_human_traj[i](1), dis_scaler_set[i], min_distance, max_distance, marker_pub);
            // drawPoints(mr_goal_set[i], "odom", "red_points" + std::to_string(i), i, marker_pub);
            // visPtr_->publishPolygon(polygons_set[i+1], "odom", "polygon_" + std::to_string(i+1), i+1, marker_pub);
            // visPtr_->publishAnnulus(i+1, intervals_set[i+1], sampled_human_traj[i+1](0), sampled_human_traj[i+1](1), dis_scaler_set[i+1], min_distance, max_distance, marker_pub);
            // drawPoints(mr_goal_set[i+1], "odom", "red_points" + std::to_string(i+1), i+1, marker_pub);
            initTrajPlanner_obj.reset(new ECBSPlanner(polys_inflat_ecbs, hPolys_ht_sets, start_states, goal_states, param, false));
            if (!initTrajPlanner_obj.get()->update(param.log)) {
              index++;
              if (index == 100) {
                goto my_label;
              }
              goto my_label;
            }
            else {
              ROS_WARN("Solved successfully after relaxing homotopic constraint!");
            }
          }
          for (int i = 0; i < initTrajPlanner_obj->initTraj.size(); i++) {
            for (int j = 0; j < initTrajPlanner_obj->initTraj[i].size(); j++) {
              ref_paths[i].push_back({initTrajPlanner_obj->initTraj[i][j].x(), initTrajPlanner_obj->initTraj[i][j].y(), initTrajPlanner_obj->initTraj[i][j].yaw()});
            }
          }
          st_time_cons.push_back({ref_paths[0].size() - 1, human_traj_index[i + 1] * 0.1});
        }

        std::vector<double> time_profile;
        std::vector<int> ref_index;
        calTimeAndMapping(st_time_cons, time_profile, ref_index);
        ref_index.push_back(human_traj.size() - 1);
        timer_step.stop();
        ROS_INFO_STREAM("ECBS Planner runtime: " << timer_step.elapsedSeconds());
        for (int i = 0; i < ref_paths.size(); i++) {
          DrawCorasePathRivz(ref_paths[i], i);
        }

      // std::vector<std::vector<std::vector<double>>> test_view(num_robot);
      // for (int i = 0; i < view_region_hyper_set.size(); i++) {
      //   for (int j = 0; j < 1; j++) {
      //     for (int k = 0; k < view_region_hyper_set[i][j].size(); k++) {
      //       auto result = view_region_hyper_set[i][j][k][0] * ref_paths[i][0](0) + view_region_hyper_set[i][j][k][1] * ref_paths[i][0](1) - view_region_hyper_set[i][j][k][2];
      //     }
      //   }
      // }
      timer_step_ig.stop();
      ROS_INFO_STREAM("Formation angles solving time: " << timer_step_ig.elapsedSeconds());
      solving_time.push_back(timer_step_ig.elapsedSeconds());
      /*------------------- compute the safe corridors -------------------*/
      env->polygons() = polys_inflat;
      for (int i = 0; i < ref_paths[0].size(); i++) {
        human_traj_.push_back({human_traj[ref_index[i]].x(), human_traj[ref_index[i]].y(), human_traj[ref_index[i]].theta()});
      }
      // human_traj_ = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/wang/3/human_traj50" + std::to_string(22) + ".yaml");

      std::vector<std::vector<std::vector<double>>> plan_sert = InsertPath(human_traj_, ref_paths.size(), ref_paths[0].size() - 1, ref_paths, 1);
      // std::vector<std::vector<Eigen::MatrixXd>> hPolys_sets(num_robot);
      std::vector<std::vector<std::vector<std::vector<double>>>> hPolys_sets(num_robot);
      std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> keyPts;
      // std::vector<std::vector<Eigen::Vector2d>> sfc;
      std::vector<std::vector<std::vector<math::Vec2d>>> corri_vertices_sets;
      std::vector<std::vector<std::vector<double>>> opt_theta_set(num_robot);
      for (int i = 0; i < ref_paths.size(); i++) {
        // std::vector<Eigen::MatrixXd> hPolys;
        std::vector<std::vector<std::vector<double>>> hPolys;
        std::vector<std::vector<math::Vec2d>> corri_vertices_set;

        std::vector<Eigen::Vector2d> ref_paths_2d;
        for (int j = 0; j < ref_paths[i].size(); j++) {
          Eigen::Vector2d visible_p;
          double theta_c, t_l, t_r;
          ref_paths_2d.push_back({plan_sert[i][j][0], plan_sert[i][j][1]});
          double min_distance_ = min_distance;
          double max_distance_ = max_distance;
          do {
            env->find_visible_neighbors({human_traj[ref_index[j]].x(), human_traj[ref_index[j]].y()}, {plan_sert[i][j][0], plan_sert[i][j][1]}, min_distance_, max_distance_, visible_p, theta_c, t_l, t_r);
            min_distance_ *= 0.9;
            max_distance_ *= 0.9;
          }
          while (t_r - t_l < theta_s / 2);
          // theta_k, p(t_k), eta_k
          opt_theta_set[i].push_back({(t_r - t_l) / 2, human_traj[ref_index[j]].x(), human_traj[ref_index[j]].y(), cos((t_l + t_r) / 2), sin((t_l + t_r) / 2)});
        }
        env->generateSFC(ref_paths_2d, 5.0, hPolys, keyPts, corri_vertices_set);
        hPolys_sets[i] = hPolys;
        corri_vertices_sets.push_back(corri_vertices_set);
        // env->visCorridor(hPolys, hPolyPub);
        // visPtr_->visualize_pairline(keyPts, "keyPts");
        // env->hPolys2Polys(hPolys, sfc);
        // visPtr_->publishPolygon(sfc, "odom", "safe_corridors", marker_pub); 
      }
      // ROS_INFO_STREAM("Corridor generation runtime: " << timer_step.elapsedSeconds());
      // for (int i = 0; i < corri_vertices_sets.size(); i++) {
      //     for (int j = 0; j < corri_vertices_sets[i].size(); j++) {
      //     auto color_poly = visualization::Color::Magenta;
      //     color_poly.set_alpha(0.1);
      //     visualization::PlotPolygon(math::Polygon2d(corri_vertices_sets[i][j]), 0.1, color_poly, 0, "Corridor" + std::to_string(i) + std::to_string(j));
      //   }
      // }
      // visualization::Trigger();  

      /*------------------- Formulate NLP problem and solving it to generate trajectory for the robot team -------------------*/
      std::vector<std::vector<std::vector<double>>>& plan_set = plan;
      std::vector<std::vector<double>> curve_set(num_robot);
      std::vector<double> initial_theta_set;
      std::vector<std::vector<int>> soft_sector_cons(num_robot, std::vector<int>(st_time_cons.size(), 0));
      std::vector<std::vector<int>> refined_index_set(num_robot);
      mr_goal_set.clear(); 
      init_formation_angle = 0.0;
      for (int i = 0; i < ref_paths[0].size(); i++) {
        std::vector<Eigen::Vector2d> points;
        center = {human_traj_[i][0], human_traj_[i][1]};
        double human_theta = human_traj_[i][2];
        if (!env->find_feasible_angle(center, init_formation_angle, min_distance, max_distance, fixed_angle_set)) {
          for (int j = 0; j < num_robot; ++j) {
            Eigen::Vector2d point = {0.0, 0.0};
            points.push_back(point);
          }
        }
        else {
          for (int j = 0; j < num_robot; ++j) {
            auto point = calculateEndpoint(center, 4, init_formation_angle + fixed_angle_set[j]);
            points.push_back(point);
          }
        }
        // env->find_desired_points(center_prev, center, 0.0, human_theta, 4, fixed_angle_set, points);
        mr_goal_set.push_back(points);
      }
      MPCPlanner_obj.reset(new MPCPlanner(human_traj_, hPolys_sets, time_profile, st_time_cons, view_region_hyper_set, opt_theta_set, ref_paths, param, initial_theta_set, soft_sector_cons, mr_goal_set));
      timer_step.reset();
      timer_step_to.reset();
      while (!MPCPlanner_obj.get()->update(param.log, plan_set, soft_sector_cons, refined_index_set)) {
        // return -1;
        ROS_WARN("Relax visible region constraints!"); 
        for (int i = 0; i < time_profile.size(); i++) {
          time_profile[i] *= 2;
        }
        MPCPlanner_obj.reset(new MPCPlanner(human_traj_, hPolys_sets, time_profile, st_time_cons, view_region_hyper_set, opt_theta_set, ref_paths, param, initial_theta_set, soft_sector_cons, mr_goal_set));
        timer_step.stop();
        // if (timer_step.elapsedSeconds() > 60) {
        //   index++;
        //   if (index == 100) {
        //     goto my_label;
        //   }
        //   goto my_label;
        // }
      }
      timer_step_to.stop();
      solving_time.push_back(timer_step_to.elapsedSeconds());
      // /* refine the trajectories */
      // for (int i = 0; i < plan_set.size(); i++) {
      //   DrawFGTrajRivz(plan_set[i], i, ref_paths[0].size());
      // }
      // timer_step.reset();
      // for (int i = 0; i < num_robot; i++) {
      //   for (int j = 0; j < plan_set[i].size(); j++) {
      //     timer_step.stop();
      //     if (timer_step.elapsedSeconds() > 120) {
      //       break;
      //     }
      //     curve_set[i].push_back(fabs(plan_set[i][j][3] / plan_set[i][j][4]));
      //     if (fabs(plan_set[i][j][3] / plan_set[i][j][4]) < 1.0 && j > 10) {
      //       ROS_WARN("Sharp turn detected! Try to relax the visibility constraint to smooth the trajectory!"); 
      //       int result = findClosestIndex(st_time_cons, j, i, refined_index_set);
      //       soft_sector_cons[i][result] = 1;
      //       MPCPlanner_obj.reset(new MPCPlanner(human_traj_, hPolys_sets, time_profile, st_time_cons, view_region_hyper_set, opt_theta_set, ref_paths, param, initial_theta_set, soft_sector_cons, mr_goal_set));
      //       if (MPCPlanner_obj.get()->update(param.log, plan_set, soft_sector_cons, refined_index_set)) {
      //         ROS_WARN("Refine trajectroy succeed!"); 
      //       }
      //       for (int i = 0; i < plan_set.size(); i++) {
      //         DrawFGTrajRivz(plan_set[i], i, ref_paths[0].size());
      //       }
      //     }
      //   }
      // }

      // ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());
      
      // delete_yaml("/home/weijian/Heterogeneous_formation/src/vis_formation_planner/traj_result/fg_plan.yaml");
      for (int i = 0; i < plan_set.size(); i++) {
        DrawFGTrajRivz(plan_set[i], i, ref_paths[0].size());
      }
      // std::vector<std::pair<double, double>> goal_sets;
      // for (int robot_ind = 0; robot_ind < ref_paths.size(); robot_ind++) {
      //   double goal_x = initTrajPlanner_obj_all->mission.goalState[robot_ind][0];
      //   double goal_y = initTrajPlanner_obj_all->mission.goalState[robot_ind][1];
      //   goal_sets.push_back(std::make_pair(goal_x, goal_y));
      // }
      double human_dis = 0.0; 
      double duration = 0.0;
      for (int i  = 1; i < human_traj_.size(); i++) {
        human_dis += hypot(human_traj_[i][0] - human_traj_[i-1][0], human_traj_[i][1] - human_traj_[i-1][1]);
      }
      for (int i = 0; i < time_profile.size(); i++) {
        duration += time_profile[i];
      }
      double human_vel_avg = human_dis / duration;
      writeVectorsToYaml(obstacle, "/home/weijian/VisFormation/src/vis_formation_planner/result/obstacle_demo" + std::to_string(num_obs[obs_num_ind]) + std::to_string(index) + ".yaml");
      writeVectorsToYaml(human_traj_, "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/3/human_traj_comp" + std::to_string(num_obs[obs_num_ind]) + std::to_string(index) + ".yaml");
      writeVectorToYAML(time_profile, "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/3/_time_profile" + std::to_string(num_obs[obs_num_ind]) + std::to_string(index) + ".yaml");
      writeVectorToYAML(solving_time, "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/3/_solving_time" + std::to_string(num_obs[obs_num_ind]) + std::to_string(index) + ".yaml");
      writeTrajectoryToYAML_FG(plan_set, ref_paths[0].size(), "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/3/fg_comp" + std::to_string(num_obs[obs_num_ind]) + std::to_string(index) + ".yaml");
      r.sleep();
      index++;
      if (index == 100) {
        break;
      }
      goto my_label;
    }
      }
    }
	return 0;
}