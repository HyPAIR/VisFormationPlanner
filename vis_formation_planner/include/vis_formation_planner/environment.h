//
// Created by weijian on 17/11/23.
//

#ifndef SRC_ENVIRONMENT_H
#define SRC_ENVIRONMENT_H
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>

#include "math/polygon2d.h"
#include "planner_config.h"

#include "decomp_geometry/polyhedron.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include "vis_formation_planner/sdlp.hpp"

namespace vis_formation_planner {

class Environment {
public:
  explicit Environment(std::shared_ptr<PlannerConfig> config): config_(std::move(config)) {}

  std::vector<math::Polygon2d> &polygons() { return polygons_; }
  std::vector<math::Vec2d> &points() { return points_; };
  std::vector<double> &heights() {return heights_; };

  struct EquationSystemResult {
    bool hasSolution;
    double x;
    double y;
  };

  bool CheckPoseCollision(double time, math::Pose pose) const;

  bool CheckVerticeCollision(double time, math::Pose pose) const;

  bool CheckHomotopyConstraints(double time, math::Pose pose, const std::vector<std::vector<double>> corridor_sets) const;

  bool CheckSpatialEnvelopes(double time, math::Pose pose) const;

  bool GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const;

  bool CheckBoxCollision(double time, const math::AABox2d &box) const;

  void UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap);

  std::vector<math::Vec2d> InflateObstacle(const std::vector<math::Vec2d>& vertices, double inflationRadius);

  bool checkRayValid(const Eigen::Vector2d& p, const Eigen::Vector2d& center);

  EquationSystemResult SolveEquationSystem(double a1, double b1, double c1, double a2, double b2, double c2);

  bool find_visible_regions(const bool& recalculate, const int& index, const Eigen::Vector2d& center, const double initial_angle, double theta_neighbor, double min_distance, double max_distance, std::vector<double>& dis_scaler, double theta_s, std::vector<std::pair<double, double>>& visible_regions, const std::vector<Eigen::Vector2d>& pt_prev);

  bool find_feasible_angle(const Eigen::Vector2d& center, double& init_formation_angle, const double min_distance, const double max_distance, const std::vector<double>& fixed_angle_set);

  bool find_feasible_points(const Eigen::Vector2d& center_prev, const Eigen::Vector2d& center, const double& human_theta_prev, const double& human_theta, const double& distance, const std::vector<double>& fixed_angle_set, std::vector<Eigen::Vector2d>& points);
  
  bool find_desired_points(
                          const Eigen::Vector2d& center_prev, 
                          const Eigen::Vector2d& center, 
                          const double& human_theta_prev, 
                          const double& human_theta, 
                          const double& distance, 
                          const std::vector<double>& fixed_angle_set, 
                          std::vector<Eigen::Vector2d>& points);

  bool inside_neighbor_area(const int& index, const Eigen::Vector2d&p_center, const std::vector<Eigen::Vector2d>& pts_prev, const double& neighbor_dis); 

  void find_visible_neighbors_(const int& index, const Eigen::Vector2d& center, const Eigen::Vector2d& seed, const double& desired_min_dis, const double& desired_max_dis, Eigen::Vector2d& visible_p, double& theta, double& t_l, double& t_r, const std::vector<Eigen::Vector2d>& pt_prev);
  
  void find_visible_neighbors(const Eigen::Vector2d& center, const Eigen::Vector2d& seed, const double& desired_min_dis, const double& desired_max_dis, Eigen::Vector2d& visible_p, double& theta, double& t_l, double& t_r);

  void generate_visible_regions(const std::vector<Eigen::Vector2d>& targets, double previous_angle, std::vector<std::vector<std::pair<double, double>>>& visible_ps, std::vector<double>& thetas);

  void compute2DViewRegion( const Eigen::Vector2d& inside_pt, const Eigen::Vector2d& target, double dis_low, double dis_upper, double alpha_low, double alpha_upper, std::vector<Eigen::Vector2d>& vert_set, Eigen::MatrixXd& view_region, std::vector<std::vector<double>>& view_region_hyper_set);

  void compressPoly(Eigen::MatrixXd& poly, double dx);

  bool findInterior(const Eigen::MatrixXd &hPoly, Eigen::Vector2d &interior);

  double findInteriorDist(const Eigen::MatrixXd &hPoly, Eigen::Vector2d &interior);

  bool filterCorridor(std::vector<Eigen::MatrixXd>& hPolys);

  void generateSFC(const std::vector<Eigen::Vector2d>& path, const double bbox_width, std::vector<std::vector<std::vector<double>>>& hPolys, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& keyPts, std::vector<std::vector<math::Vec2d>>& poly_vertices_set);

  void visCorridor(const vec_E<Polyhedron2D>& polyhedra, const ros::Publisher& hPolyPub);

  void visCorridor(const std::vector<Eigen::MatrixXd>& hPolys, const ros::Publisher& hPolyPub);

  void hPolys2Polys(const std::vector<Eigen::MatrixXd>& hPolys, std::vector<std::vector<Eigen::Vector2d>>& polygons);

  std::vector<Eigen::Vector2d> interpolatePoints(const Eigen::Vector2d& start, const Eigen::Vector2d& end, int num_points);

  bool visible_regions_feasible(const int& ind, const std::vector<double>& dis_scaler, const std::vector<Eigen::Vector2d>& intervals, const std::vector<Eigen::Vector2d>& points, int& tune_index);

private:
  std::shared_ptr<PlannerConfig> config_;
  std::vector<math::Polygon2d> polygons_;
  std::vector<math::Vec2d> points_;
  std::vector<double> heights_;
};

}

#endif //SRC_ENVIRONMENT_H
