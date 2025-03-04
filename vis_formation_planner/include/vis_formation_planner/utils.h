#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include "vis_formation_planner/vis_formation_planner.h"
#include "vis_formation_planner/math/generate_obs.h"
#include "vis_formation_planner/visualization/plot.h"
#include "vis_formation_planner/yaml_all.h"
// #include "traj_tracking/matplotlibcpp.h"
#include "vis_formation_planner/forward_kinematics.h"
#include <nav_msgs/Path.h>
#include <random>
#include "vis_formation_planner/visualization/visualization.h"
#include "vis_formation_planner/robot_angle_optimizaer.h"
#include "to_solver/init_traj_planner.hpp"

using namespace vis_formation_planner;
void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<vis_formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub);
void DrawCorasePathRivz(const std::vector<Eigen::Vector3d> path, int robot_index);
void DrawFGTrajRivz(const std::vector<std::vector<double>> traj, int robot_index, int traj_length);
FullStates traj2fullstates(Trajectory_temp traj);
std::vector<std::vector<std::vector<double>>> InsertPath(const std::vector<std::vector<double>>& human_traj, int qn, int M, std::vector<std::vector<Eigen::Vector3d>> initTraj, const int& discrete);
// void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<vis_formation_planner::PlannerConfig> config,
//       int robot_index, ros::Publisher path_pub, double inflat);
void generateRegularPolygon(const double cx, const double cy, const double r, const int k, 
  std::vector<std::vector<double>>& vertice_initial_set);
void PlotPose(std::shared_ptr<vis_formation_planner::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type, const double& xo, const double& yo);
double getRandomDouble(double min, double max);
double getRandomAngle();
TrajectoryPoint generateRandomnObstacle();
void GenerateRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, std::vector<std::vector<double>>& obstacle,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat, std::vector<math::Polygon2d>& polys_inflat_ecbs,
std::vector<math::Polygon2d>& polys_inflat_ecbs_exe, std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs);
bool ReadRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, const std::string& file_path,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat, std::vector<math::Polygon2d>& polys_inflat_ecbs,
std::vector<math::Polygon2d>& polys_inflat_ecbs_exe,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
);
double normalizeTo2Pi(double angle);
std::vector<std::vector<std::pair<double, double>>> normalizeAngleRanges(
    const std::vector<std::vector<std::pair<double, double>>>& angleRanges);
Eigen::Vector2d calculateEndpoint(const Eigen::Vector2d& center, double distance, double angle);
void drawPoints(const std::vector<Eigen::Vector2d>& distancesAndAngles,
                const std::string& frame_id,
                const std::string& namespace_id,
                const int& id,
                ros::Publisher& marker_pub);
std::vector<size_t> findIndices(const std::vector<math::Pose> traj);
void calTimeAndMapping(const std::vector<std::pair<int, double>>& st_time_cons, std::vector<double>& time_profile, std::vector<int>& ref_index);
int findClosestIndex(const std::vector<std::pair<int, double>>& vec, int index, int robot_ind, std::vector<std::vector<int>>& refined_index_set);