//
// Created by weijian on 17/11/23.
//

#include "vis_formation_planner/environment.h"
#include <costmap_2d/cost_values.h>

namespace vis_formation_planner {

  bool Environment::CheckBoxCollision(double time, const math::AABox2d &box) const {
    // TODO: reimplement using R-Tree
    for(auto &polygon: polygons_) {
      if(polygon.HasOverlap(math::Box2d(box))) {
        return true;
      }
    }

    for(auto &point: points_) {
      if(box.IsPointIn(point)) {
        return true;
      }
    }

    return false;
  }

  bool Environment::CheckPoseCollision(double time, math::Pose pose) const {
    auto discs = config_->vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());
    double wh = config_->vehicle.disc_radius * 2;
    for(int i = 0; i < discs.size() / 2; i++) {
      if(CheckBoxCollision(time, math::AABox2d({discs[i * 2], discs[i * 2 + 1]}, wh, wh))) {
        return true;
      }
    }

    return false;
  }

  bool Environment::CheckVerticeCollision(double time, math::Pose pose) const {
    auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
    if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, 2.0 + 1.2, config_->vehicle.offset + 1.2))) {
      return true;
    }
    return false;
  }


  bool Environment::CheckHomotopyConstraints(double time, math::Pose pose, const std::vector<std::vector<double>> corridor_sets) const {
    for (int i = 0; i < corridor_sets.size(); i++) {
      if (pose.x() >= corridor_sets[i][0] && pose.x() <= corridor_sets[i][2]
          && pose.y() >= corridor_sets[i][1] && pose.y() <= corridor_sets[i][3]) {
            return true;
          }
    }
    return false;
  }

  bool Environment::CheckSpatialEnvelopes(double time, math::Pose pose) const {
    auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
    if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, sqrt(2) * (2.0 + 1.2), sqrt(2) * (config_->vehicle.offset + 1.2)))) {
      return true;
    }
    return false;
  }

  void Environment::UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap) {
    points_.clear();
    // 遍历每一个网格
    for(int i = 0; i < costmap->getSizeInCellsX()-1; i++) {
      for(int j = 0; j < costmap->getSizeInCellsY()-1; j++) {
        if(costmap->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE) {
          double obs_x, obs_y;
          costmap->mapToWorld(i,j, obs_x, obs_y);
          points_.emplace_back(obs_x, obs_y);
        }
      }
    }
  }

  bool Environment::GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const {
    double ri = radius;
    // 或许应该是math::AABox2d bound({x-ri, y-ri}, {x+ri, y+ri});
    math::AABox2d bound({-ri, -ri}, {ri, ri});

    if(CheckBoxCollision(time, bound)) {
      // initial condition not satisfied, involute to find feasible box
      int inc = 4;
      double real_x, real_y;

      do {
        int iter = inc / 4;
        uint8_t edge = inc % 4;

        real_x = x;
        real_y = y;
        if(edge == 0) {
          real_x = x - iter * 0.05;
        } else if(edge == 1) {
          real_x = x + iter * 0.05;
        } else if(edge == 2) {
          real_y = y - iter * 0.05;
        } else if(edge == 3) {
          real_y = y + iter * 0.05;
        }

        inc++;
        bound = math::AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
      } while(CheckBoxCollision(time, bound) && inc < config_->corridor_max_iter);
      if(inc > config_->corridor_max_iter) {
        return false;
      }

      x = real_x;
      y = real_y;
    }

    int inc = 4;
    std::bitset<4> blocked;
    double incremental[4] = {0.0};
    double step = radius * 0.2;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;
      inc++;

      if (blocked[edge]) continue;

      incremental[edge] = iter * step;

      math::AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                  {ri + incremental[1], ri + incremental[3]});

      if (CheckBoxCollision(time, test.Offset({x, y})) || incremental[edge] >= config_->corridor_incremental_limit) {
        incremental[edge] -= step;
        blocked[edge] = true;
      }
    } while (!blocked.all() && inc < config_->corridor_max_iter);
    if (inc > config_->corridor_max_iter) {
      return false;
    }

    result = {{x - incremental[0], y - incremental[2]},
              {x + incremental[1], y + incremental[3]}};
    return true;
  }

  std::vector<math::Vec2d> Environment::InflateObstacle(const std::vector<math::Vec2d>& vertices, double inflationRadius) {
      std::vector<math::Vec2d> inflatedObstacle;
      std::vector<math::Vec2d> inflatedPolygon;

      // Iterate over each vertex of the polygon
      for (size_t i = 0; i < vertices.size(); i++) {
          // Get the current, previous, and next vertices
          math::Vec2d currentVertex = vertices[i];
          math::Vec2d prevVertex = vertices[(i + vertices.size() - 1) % vertices.size()];
          math::Vec2d nextVertex = vertices[(i + 1) % vertices.size()];

          // Calculate the direction vectors of the adjacent edges
          double prevEdgeX = currentVertex.x() - prevVertex.x();
          double prevEdgeY = currentVertex.y() - prevVertex.y();
          double nextEdgeX = nextVertex.x() - currentVertex.x();
          double nextEdgeY = nextVertex.y() - currentVertex.y();

          // Normalize the direction vectors
          double prevEdgeLength = std::sqrt(prevEdgeX * prevEdgeX + prevEdgeY * prevEdgeY);
          double nextEdgeLength = std::sqrt(nextEdgeX * nextEdgeX + nextEdgeY * nextEdgeY);
          prevEdgeX /= prevEdgeLength;
          prevEdgeY /= prevEdgeLength;
          nextEdgeX /= nextEdgeLength;
          nextEdgeY /= nextEdgeLength;

          // Calculate the perpendicular vectors of the adjacent edges
          double prevPerpendicularX = -prevEdgeY;
          double prevPerpendicularY = prevEdgeX;
          double nextPerpendicularX = -nextEdgeY;
          double nextPerpendicularY = nextEdgeX;

          // Calculate the inflated vertices
          double prevX = currentVertex.x() + inflationRadius * (prevPerpendicularX);
          double prevY = currentVertex.y() + inflationRadius * (prevPerpendicularY);
          double nextX = currentVertex.x() + inflationRadius * (nextPerpendicularX);
          double nextY = currentVertex.y() + inflationRadius * (nextPerpendicularY);
          // Define the coefficients of the equation system
          double a1 = prevX - currentVertex.x();
          double b1 = prevY - currentVertex.y();
          double c1 = prevX * (prevX - currentVertex.x()) + prevY * (prevY - currentVertex.y());
          double a2 = nextX - currentVertex.x();
          double b2 = nextY - currentVertex.y();
          double c2 = nextX * (nextX - currentVertex.x()) + nextY * (nextY - currentVertex.y());
          // Solve the equation system
          EquationSystemResult result = SolveEquationSystem(a1, b1, c1, a2, b2, c2);

          // Add the inflated vertex to the inflated polygon
          inflatedPolygon.push_back({result.x, result.y});
      }

      return inflatedPolygon;
  }

  bool Environment::checkRayValid(const Eigen::Vector2d& p, const Eigen::Vector2d& center) {
      struct LineSegment {
          Eigen::Vector2d start;
          Eigen::Vector2d end;
      };
      LineSegment segment = {{p(0), p(1)}, {center(0), center(1)}}; 

      // Lambda for cross product
      auto cross = [](const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
          return v1.x() * v2.y() - v1.y() * v2.x();
      };

      // Updated isSegmentsIntersect
      auto isSegmentsIntersect = [&](const LineSegment& s1, const LineSegment& s2) -> bool {
          Eigen::Vector2d p = s1.start, q = s1.end;
          Eigen::Vector2d r = s2.start, s = s2.end;

          Eigen::Vector2d pq = q - p, pr = r - p, ps = s - p, rs = s - r;
          double cross1 = cross(pq, pr);
          double cross2 = cross(pq, ps);
          double cross3 = cross(rs, p - r);
          double cross4 = cross(rs, q - r);

          if (std::abs(cross1) < 1e-9 && std::abs(cross2) < 1e-9) {
              double dot1 = (r - p).dot(q - p);
              double dot2 = (s - p).dot(q - p);
              return !(dot1 < 0 && dot2 < 0);
          }

          return (cross1 * cross2 <= 0 && cross3 * cross4 <= 0);
      };

      for (const auto& polygon : polygons_) {
          const auto& points = polygon.points();
          int num_points = points.size();
          for (int j = 0; j < num_points; j++) {
              Eigen::Vector2d a = {points[j].x(), points[j].y()};
              Eigen::Vector2d b = {points[(j + 1) % num_points].x(), points[(j + 1) % num_points].y()};
              LineSegment edge = {a, b};

              if (isSegmentsIntersect(segment, edge)) {
                  return false;
              }
          }
      }

      auto isPointInPolygon = [&](const Eigen::Vector2d& point) -> bool {
          for (const auto& polygon : polygons_) {
              const auto& points = polygon.points();
              int num_points = points.size();
              int crossings = 0;

              for (int j = 0; j < num_points; ++j) {
                  Eigen::Vector2d a = {points[j].x(), points[j].y()};
                  Eigen::Vector2d b = {points[(j + 1) % num_points].x(), points[(j + 1) % num_points].y()};

                  if ((a.y() > point.y()) != (b.y() > point.y())) {
                      double t = (point.y() - a.y()) / (b.y() - a.y());
                      double x_intersection = a.x() + t * (b.x() - a.x());
                      if (point.x() < x_intersection) crossings++;
                  }
              }
              if (crossings % 2 == 1) return true;
          }
          return false;
      };

      if (isPointInPolygon(segment.start) || isPointInPolygon(segment.end)) {
          return false;
      }

      return true;
  }

  Environment::EquationSystemResult Environment::SolveEquationSystem(double a1, double b1, double c1, double a2, double b2, double c2) {
      double determinant = a1 * b2 - a2 * b1;

      Environment::EquationSystemResult result;
      result.hasSolution = false;

      if (determinant != 0) {
          result.hasSolution = true;
          result.x = (c1 * b2 - c2 * b1) / determinant;
          result.y = (a1 * c2 - a2 * c1) / determinant;
      }

      return result;
  }

  bool is_crossing_boundary(double left_limit, double right_limit) {
      return right_limit < left_limit; // right_limit小于left_limit表示跨越边界
  }
  bool Environment::find_visible_regions( const bool& recalculate,
                                          const int& index,
                                          const Eigen::Vector2d& center,
                                          const double initial_angle,
                                          double theta_neighbor,
                                          double min_distance,
                                          double max_distance,
                                          std::vector<double>& dis_scaler,
                                          double theta_s,
                                          std::vector<std::pair<double, double>>& visible_regions,
                                          const std::vector<Eigen::Vector2d>& pts_prev) {
      double neighbor_area_radius = 10;
      visible_regions.clear();
      if (recalculate) {
        theta_s = 0.0;
      }
      double d_theta = 0.1; // Angle step size
      bool found_region = false;

      auto normalizeAngle = [](double angle, double range_min, double range_max) {
          double range = range_max - range_min;
          while (angle < range_min) angle += range;
          while (angle >= range_max) angle -= range;
          return angle;
      };
      bool final_adjust = false;
      while (!found_region) {
          double cumulative_visible_angle = 0.0;

          double left_limit = normalizeAngle(initial_angle - theta_neighbor, -M_PI, M_PI);
          double right_limit = normalizeAngle(initial_angle + theta_neighbor, -M_PI, M_PI);

          // Left-side boundary search with pre-splitting of intervals
          std::vector<std::pair<double, double>> left_intervals;

          if (is_crossing_boundary(left_limit, right_limit)) {
              left_intervals.emplace_back(left_limit, M_PI);   // 第一段
              left_intervals.emplace_back(-M_PI, right_limit); // 第二段
          } else {
              left_intervals.emplace_back(left_limit, right_limit);
          }

          for (const auto& interval : left_intervals) {
              double start = interval.first;
              double end = interval.second;
              double t_l = start;
              for (double current_angle = start; current_angle < end; current_angle += d_theta) {
                  double normalized_angle = normalizeAngle(current_angle, -M_PI, M_PI);
                  Eigen::Vector2d p_min = center, p_max = center, p_center = center;
                  p_min.x() += dis_scaler[index] * min_distance * cos(normalized_angle);
                  p_min.y() += dis_scaler[index] * min_distance * sin(normalized_angle);
                  p_max.x() += dis_scaler[index] * max_distance * cos(normalized_angle);
                  p_max.y() += dis_scaler[index] * max_distance * sin(normalized_angle);
                  p_center.x() += dis_scaler[index] * ((max_distance + min_distance) / 2) * cos(normalized_angle);
                  p_center.y() += dis_scaler[index] * ((max_distance + min_distance) / 2) * sin(normalized_angle);
                  // occluded by the obstacles / do not expand to neighboring areas
                  if (!checkRayValid(p_min, center) || !checkRayValid(p_max, center) || !inside_neighbor_area(index, p_center, pts_prev, neighbor_area_radius)) {
                      if (t_l < normalized_angle && fabs(t_l - normalized_angle) > M_PI / 18) {
                          visible_regions.emplace_back(t_l, normalized_angle - d_theta);
                          cumulative_visible_angle += normalized_angle - t_l;
                      }
                      t_l = normalized_angle + d_theta;
                  }
              }

              // Finalize remaining interval
              if (t_l < end && fabs(t_l - end) > M_PI / 18) {
                  visible_regions.emplace_back(t_l, end);
                  cumulative_visible_angle += end - t_l;
              }
          }
          if (cumulative_visible_angle >= theta_s) {
            found_region = true;
            std::cout << visible_regions.size() << " feasible regions found!" << std::endl;
            return true;
          } 
          // else {
          //   std::cout << "No visible regions found! Too complex environment!" << std::endl;
          //   return false;
          // }
          else {
            if (theta_neighbor > M_PI / 2) {
              std::cout << "No visible regions found! Try to adjust the distance!" << std::endl;
              dis_scaler[index] *= 0.9;
              if (dis_scaler[index] * min_distance < 1.5 && !recalculate && !final_adjust) {
                theta_s = 0.0;
                final_adjust = true;
              }
              else if (final_adjust) {
                std::cout << "No visible regions found! Too complex environment!" << std::endl;
                visible_regions.clear();
                return false;
              }
            }
            else {
              theta_neighbor *= 1.1;
              std::cout << "No visible regions found! Try to adjust the angle!" << std::endl;
            }
            visible_regions.clear();
          }
      }
  }

  bool Environment::find_feasible_angle(const Eigen::Vector2d& center, 
                                        double& init_formation_angle, 
                                        const double min_distance, 
                                        const double max_distance, 
                                        const std::vector<double>& fixed_angle_set) {
    double d_theta = 0.1; // Angle step size
    bool found_region = false;

    auto normalizeAngle = [](double angle, double range_min, double range_max) {
        double range = range_max - range_min;
        while (angle < range_min) angle += range;
        while (angle >= range_max) angle -= range;
        return angle;
    };
    bool final_adjust = false;
    double left_angle = 1e5;
    double right_angle = 1e5;
    bool left_found = false;
    bool right_found = false;
    while (!found_region) {
      for (double current_angle = init_formation_angle; current_angle < init_formation_angle + M_PI; current_angle += d_theta) {
          bool valid = true;  
          for (int i = 0; i < fixed_angle_set.size(); i++) {
            double current_robot_angle = current_angle + fixed_angle_set[i];
            for (double theta_check = current_robot_angle + M_PI / 16; theta_check > current_robot_angle - M_PI / 16; theta_check -= d_theta) {
              Eigen::Vector2d p_min = center, p_max = center, p_center = center;
              p_min.x() += min_distance * cos(theta_check);
              p_min.y() += min_distance * sin(theta_check);
              p_max.x() += max_distance * cos(theta_check);
              p_max.y() += max_distance * sin(theta_check);

              // occluded by the obstacles / do not expand to neighboring areas
              if (!checkRayValid(p_min, center) || !checkRayValid(p_max, center)) {
                valid = false;
                break;  
              }
            }
            if (!valid) break;  
          }
          if (!valid) continue;  

          left_found = true;
          left_angle = current_angle;
          break;  
      }

      for (double current_angle = init_formation_angle; current_angle > init_formation_angle - M_PI; current_angle -= d_theta) {
        bool valid = true;  
        for (int i = 0; i < fixed_angle_set.size(); i++) {
          double current_robot_angle = current_angle + fixed_angle_set[i];
          for (double theta_check = current_robot_angle + M_PI / 16; theta_check > current_robot_angle - M_PI / 16; theta_check -= d_theta) {
            Eigen::Vector2d p_min = center, p_max = center, p_center = center;
            p_min.x() += min_distance * cos(theta_check);
            p_min.y() += min_distance * sin(theta_check);
            p_max.x() += max_distance * cos(theta_check);
            p_max.y() += max_distance * sin(theta_check);
            // occluded by the obstacles / do not expand to neighboring areas
            if (!checkRayValid(p_min, center) || !checkRayValid(p_max, center)) {    
              valid = false;
              break;      
            }
          }
          if (!valid) break;  
        }
        if (!valid) continue;  
        
        right_found = true;
        right_angle = current_angle;
        break;
      }
      if (right_found && !left_found) {
        // t_l = right_angle + M_PI / 16;
        // t_r = right_angle - M_PI / 16;
        init_formation_angle = right_angle;
        return true;
      }
      else if (left_found && !right_found) {
        // t_l = left_angle + M_PI / 16;
        // t_r = left_angle - M_PI / 16;
        init_formation_angle = left_angle;
        return true;
      }
      else if (left_found && right_found){
        double smaller_angle = fabs(left_angle - init_formation_angle) > fabs(right_angle - init_formation_angle) ? left_angle : right_angle;
        // t_l = smaller_angle + M_PI / 16;
        // t_r = smaller_angle - M_PI / 16;
        init_formation_angle = smaller_angle;
        return true;
      }
      else {
        std::cout << "No visible regions found! Try to adjust the angle!" << std::endl;
        return false;
      }
    }
  }

  bool Environment::find_feasible_points(
                          const Eigen::Vector2d& center_prev, 
                          const Eigen::Vector2d& center, 
                          const double& human_theta_prev, 
                          const double& human_theta, 
                          const double& distance, 
                          const std::vector<double>& fixed_angle_set, 
                          std::vector<Eigen::Vector2d>& points) {
    for (int i = 0; i < fixed_angle_set.size(); i++) {
      double angle = human_theta + fixed_angle_set[i];
      Eigen::Vector2d robot_pos = {center(0) + distance * cos(angle), center(1) + distance * sin(angle)};
      if(!checkRayValid(robot_pos, center)) {
        Eigen::Vector2d mid_pt = 0.5 * (center + robot_pos);
        if (checkRayValid(mid_pt, center)) {
          double updated_dis = distance;
          Eigen::Vector2d updated_pt = mid_pt;
          Eigen::Vector2d pt;
          do
          {
            pt = updated_pt;
            updated_dis += 0.1;
            updated_pt = {center(0) + updated_dis * cos(angle), center(1) + updated_dis * sin(angle)};
          } while (checkRayValid(updated_pt, center));
          points.push_back(pt);
        }
        else {
          Eigen::Vector2d pt = 0.5 * (center + center_prev);
          points.push_back(pt);
        }
      }
      else {
        points.push_back(robot_pos);
      }
    }                          
    return true;
  }

  bool Environment::find_desired_points(
                          const Eigen::Vector2d& center_prev, 
                          const Eigen::Vector2d& center, 
                          const double& human_theta_prev, 
                          const double& human_theta, 
                          const double& distance, 
                          const std::vector<double>& fixed_angle_set, 
                          std::vector<Eigen::Vector2d>& points) {
    for (int i = 0; i < fixed_angle_set.size(); i++) {
      double angle = fixed_angle_set[i];
      // double angle = human_theta + fixed_angle_set[i];
      Eigen::Vector2d robot_pos = {center(0) + distance * cos(angle), center(1) + distance * sin(angle)};
      points.push_back(robot_pos);
    }                          
    return true;
  }

  bool Environment::inside_neighbor_area(const int& index, const Eigen::Vector2d& p_center, const std::vector<Eigen::Vector2d>& pts_prev, const double& neighbor_dis) {
    if (pts_prev.empty()) {
      return true;
    }
    else if (hypot(p_center(0) - pts_prev[index](0), p_center(1) - pts_prev[index](1)) > neighbor_dis) {
      return false;
    }
    else {
      return true;
    }
  }

  void Environment::find_visible_neighbors_(
                           const int& index,
                           const Eigen::Vector2d& center,
                           const Eigen::Vector2d& seed,
                           const double& desired_min_dis,
                           const double& desired_max_dis,
                           Eigen::Vector2d& visible_p,
                           double& theta_c,
                           double& t_l, 
                           double& t_r,
                           const std::vector<Eigen::Vector2d>& pts_prev) {
    double neighbor_area_radius = 10;
    Eigen::Vector2d dp = seed - center;
    double theta0 = atan2(dp.y(), dp.x());
    double d_theta = 0.05;
    double desired_range = M_PI / 8; // Total range of the neighborhood

    // Initialize the left and right boundaries
    t_l = theta0;
    t_r = theta0;
    bool left_valid, right_valid;
    // Expand the left and right boundaries symmetrically
    while (t_r - t_l < desired_range) {
        // Try expanding to the left
        double new_t_l = t_l - d_theta;
        Eigen::Vector2d p_min_l = center, p_max_l = center, p_center_l = center;
        p_min_l.x() += desired_min_dis * cos(new_t_l);
        p_min_l.y() += desired_min_dis * sin(new_t_l);
        p_max_l.x() += desired_max_dis * cos(new_t_l);
        p_max_l.y() += desired_max_dis * sin(new_t_l);
        p_center_l.x() += ((desired_max_dis + desired_min_dis) / 2) * cos(new_t_l);
        p_center_l.y() += ((desired_max_dis + desired_min_dis) / 2) * sin(new_t_l);

        // Try expanding to the right
        double new_t_r = t_r + d_theta;
        Eigen::Vector2d p_min_r = center, p_max_r = center, p_center_r = center;
        p_min_r.x() += desired_min_dis * cos(new_t_r);
        p_min_r.y() += desired_min_dis * sin(new_t_r);
        p_max_r.x() += desired_max_dis * cos(new_t_r);
        p_max_r.y() += desired_max_dis * sin(new_t_r);
        p_center_r.x() += ((desired_max_dis + desired_min_dis) / 2) * cos(new_t_l);
        p_center_r.y() += ((desired_max_dis + desired_min_dis) / 2) * sin(new_t_l);

        // Check if both expansions are valid
        left_valid = checkRayValid(p_min_l, center) && checkRayValid(p_max_l, center) && 
                     inside_neighbor_area(index, p_center_l, pts_prev, neighbor_area_radius) && inside_neighbor_area(index, p_center_l, pts_prev, neighbor_area_radius);
        right_valid = checkRayValid(p_min_r, center) && checkRayValid(p_max_r, center) && 
                      inside_neighbor_area(index, p_center_r, pts_prev, neighbor_area_radius) && inside_neighbor_area(index, p_center_r, pts_prev, neighbor_area_radius);;

        if (left_valid && right_valid) {
            // Expand symmetrically
            t_l = new_t_l;
            t_r = new_t_r;
        } else if (left_valid) {
            // Expand only to the left
            t_l = new_t_l;
        } else if (right_valid) {
            // Expand only to the right
            t_r = new_t_r;
        } else {
            // Stop if neither direction can expand
            break;
        }
    }
    // if (!left_valid) t_l = t_l + d_theta;
    // if (!right_valid) t_r = t_r - d_theta;

    double theta_v = (t_l + t_r) / 2;
    visible_p = center;
    visible_p.x() += (desired_min_dis + desired_max_dis) / 2 * cos(theta_v);
    visible_p.y() += (desired_min_dis + desired_max_dis) / 2 * sin(theta_v);
    double theta = (t_r - t_l) / 2;
    // theta_c = theta < theta_clearance ? theta : theta_clearance;
    // if (theta < theta_clearance) {
    //   theta_c = theta;
    //   t_l = theta_v - theta;
    //   t_r = theta_v + theta; 
    // }
    // else {
    //   theta_c = theta_clearance;
    //   t_l = theta_v - theta_clearance;
    //   t_r = theta_v + theta_clearance; 
    // }
    // 限制在theta_clearance范围内
    // if (theta0 - t_l < theta_c) {
    //   seed = center;
    //   seed.x() += (desired_min_dis + desired_max_dis) / 2 * cos(t_l + theta_c);
    //   seed.y() += (desired_min_dis + desired_max_dis) / 2 * sin(t_l + theta_c);
    // } else if (t_r - theta0 < theta_c) {
    //   seed = center;
    //   seed.x() += (desired_min_dis + desired_max_dis) / 2 * cos(t_r - theta_c);
    //   seed.y() += (desired_min_dis + desired_max_dis) / 2 * sin(t_r - theta_c);
    // }
    return;
  }

  void Environment::find_visible_neighbors(const Eigen::Vector2d& center,
                           const Eigen::Vector2d& seed,
                           const double& desired_min_dis,
                           const double& desired_max_dis,
                           Eigen::Vector2d& visible_p,
                           double& theta_c,
                           double& t_l, 
                           double& t_r) {
    Eigen::Vector2d dp = seed - center;
    double theta0 = atan2(dp.y(), dp.x());
    double d_theta = 0.05;
    double desired_range = M_PI / 8; // Total range of the neighborhood
    // double desired_range = 0.0; // Ho's method

    // Initialize the left and right boundaries
    t_l = theta0;
    t_r = theta0;
    bool left_valid, right_valid;
    // Expand the left and right boundaries symmetrically
    while (t_r - t_l < desired_range) {
        // Try expanding to the left
        double new_t_l = t_l - d_theta;
        Eigen::Vector2d p_min_l = center, p_max_l = center;
        p_min_l.x() += desired_min_dis * cos(new_t_l);
        p_min_l.y() += desired_min_dis * sin(new_t_l);
        p_max_l.x() += desired_max_dis * cos(new_t_l);
        p_max_l.y() += desired_max_dis * sin(new_t_l);

        // Try expanding to the right
        double new_t_r = t_r + d_theta;
        Eigen::Vector2d p_min_r = center, p_max_r = center;
        p_min_r.x() += desired_min_dis * cos(new_t_r);
        p_min_r.y() += desired_min_dis * sin(new_t_r);
        p_max_r.x() += desired_max_dis * cos(new_t_r);
        p_max_r.y() += desired_max_dis * sin(new_t_r);

        // Check if both expansions are valid
        left_valid = checkRayValid(p_min_l, center) && checkRayValid(p_max_l, center);
        right_valid = checkRayValid(p_min_r, center) && checkRayValid(p_max_r, center);

        if (left_valid && right_valid) {
            // Expand symmetrically
            t_l = new_t_l;
            t_r = new_t_r;
        } else if (left_valid) {
            // Expand only to the left
            t_l = new_t_l;
        } else if (right_valid) {
            // Expand only to the right
            t_r = new_t_r;
        } else {
            // Stop if neither direction can expand
            break;
        }
    }
    // if (!left_valid) t_l = t_l + d_theta;
    // if (!right_valid) t_r = t_r - d_theta;

    double theta_v = (t_l + t_r) / 2;
    visible_p = center;
    visible_p.x() += (desired_min_dis + desired_max_dis) / 2 * cos(theta_v);
    visible_p.y() += (desired_min_dis + desired_max_dis) / 2 * sin(theta_v);
    double theta = (t_r - t_l) / 2;
    // theta_c = theta < theta_clearance ? theta : theta_clearance;
    // if (theta < theta_clearance) {
    //   theta_c = theta;
    //   t_l = theta_v - theta;
    //   t_r = theta_v + theta; 
    // }
    // else {
    //   theta_c = theta_clearance;
    //   t_l = theta_v - theta_clearance;
    //   t_r = theta_v + theta_clearance; 
    // }
    // 限制在theta_clearance范围内
    // if (theta0 - t_l < theta_c) {
    //   seed = center;
    //   seed.x() += (desired_min_dis + desired_max_dis) / 2 * cos(t_l + theta_c);
    //   seed.y() += (desired_min_dis + desired_max_dis) / 2 * sin(t_l + theta_c);
    // } else if (t_r - theta0 < theta_c) {
    //   seed = center;
    //   seed.x() += (desired_min_dis + desired_max_dis) / 2 * cos(t_r - theta_c);
    //   seed.y() += (desired_min_dis + desired_max_dis) / 2 * sin(t_r - theta_c);
    // }
    return;
  }

  // void Environment::generate_visible_regions(const std::vector<Eigen::Vector2d>& targets,
  //                                     double previous_angle,
  //                                     std::vector<std::vector<std::pair<double, double>>>& visible_regions,
  //                                     std::vector<double>& thetas) {
  //   std::vector<std::pair<double, double>> visible_region;
  //   visible_regions.clear();
  //   thetas.clear();
  //   Eigen::Vector2d visible_p;
  //   double theta = 0;
  //   int M = targets.size();
  //   double scaler = 1.0;
  //   for (int i = 0; i < M; ++i) {
  //     find_visible_regions(targets[i], previous_angle, 0.5, 2, 5, scaler, theta, visible_region);
  //     visible_regions.push_back(visible_region);
  //     thetas.push_back(theta);
  //   }
  //   return;
  // }

  // Function to normalize a vector
  Eigen::Vector2d normalize2D(const Eigen::Vector2d& vec) {
      return vec.normalized();
  }

  // Function to compute 2D view region polyhedra
  void Environment::compute2DViewRegion(
      const Eigen::Vector2d& inside_pt,
      const Eigen::Vector2d& target,
      double dis_low,
      double dis_upper,
      double alpha_low,
      double alpha_upper,
      std::vector<Eigen::Vector2d>& vert_set,
      Eigen::MatrixXd& view_region,
      std::vector<std::vector<double>>& view_region_hyper
      ) {
      // Compute points of the view region
      Eigen::Vector2d p1, p2, p3, p4, p5, v1, v2, v3, v4, v5, v;
      p1 = target + dis_low * Eigen::Vector2d(cos(alpha_low), sin(alpha_low));
      p2 = target + dis_upper * Eigen::Vector2d(cos(alpha_low), sin(alpha_low));
      p3 = target + dis_upper * Eigen::Vector2d(cos((alpha_low + alpha_upper) / 2), sin((alpha_low + alpha_upper) / 2));
      p4 = target + dis_upper * Eigen::Vector2d(cos(alpha_upper), sin(alpha_upper));
      p5 = target + dis_low * Eigen::Vector2d(cos(alpha_upper), sin(alpha_upper));
      vert_set = {p1, p2, p3, p4, p5};
      // Compute normal vectors
      v = Eigen::Vector2d(-(p2 - p1).y(), (p2 - p1).x()); // Perpendicular to edge p1-p2
      v1 = normalize2D(v);
      if (v1.dot(inside_pt) - v1.dot(p1) > 0) {
        v1 = -v1;
      }

      v = Eigen::Vector2d(-(p3 - p2).y(), (p3 - p2).x()); // Perpendtargeticular to edge p2-p3
      v2 = normalize2D(v);
      if (v2.dot(inside_pt) - v2.dot(p2) > 0) {
        v2 = -v2;
      }

      v = Eigen::Vector2d(-(p4 - p3).y(), (p4 - p3).x()); // Perpendicular to edge p3-p4
      v3 = normalize2D(v);
      if (v3.dot(inside_pt) - v3.dot(p3) > 0) {
        v3 = -v3;
      }

      v = Eigen::Vector2d(-(p5 - p4).y(), (p5 - p4).x()); // Perpendicular to edge p4-p5
      v4 = normalize2D(v);
      if (v4.dot(inside_pt) - v4.dot(p4) > 0) {
        v4 = -v4;
      }

      v = Eigen::Vector2d(-(p1 - p5).y(), (p1 - p5).x()); // Perpendicular to edge p5-p1
      v5 = normalize2D(v);
      if (v5.dot(inside_pt) - v5.dot(p5) > 0) {
        v5 = -v5;
      }

      // Store view region in the matrix
      view_region.resize(4, 5); // 2D normals + 2D points = 4 rows, 5 columns for each vertex
      view_region.block<2, 1>(0, 0) = v1; view_region.block<2, 1>(2, 0) = p1;
      view_region.block<2, 1>(0, 1) = v2; view_region.block<2, 1>(2, 1) = p2;
      view_region.block<2, 1>(0, 2) = v3; view_region.block<2, 1>(2, 2) = p3;
      view_region.block<2, 1>(0, 3) = v4; view_region.block<2, 1>(2, 3) = p4;
      view_region.block<2, 1>(0, 4) = v5; view_region.block<2, 1>(2, 4) = p5;
      for (int j = 0; j < view_region.cols(); j++) {
        Eigen::Vector2d a = view_region.col(j).head(2); 
        Eigen::Vector2d p = view_region.col(j).tail(2);     
        double b = a.dot(p);   
        view_region_hyper.push_back({a(0), a(1), b});       
      }
  }

  void Environment::compressPoly(Eigen::MatrixXd& poly, double dx) {
    for (int i = 0; i < poly.cols(); ++i) {
      poly.col(i).tail(3) = poly.col(i).tail(3) - poly.col(i).head(3) * dx;
    }
  }

  bool Environment::findInterior(const Eigen::MatrixXd &hPoly, Eigen::Vector2d &interior) {
    int m = hPoly.cols();

    Eigen::MatrixXd A(m, 3); 
    Eigen::VectorXd b(m), c(3), x(3);
    A.leftCols<2>() = hPoly.topRows<2>().transpose();
    A.rightCols<1>().setConstant(1.0); 
    b = hPoly.topRows<2>().cwiseProduct(hPoly.bottomRows<2>()).colwise().sum().transpose();
    c.setZero();
    c(2) = -1.0; 
    double minmaxsd = sdlp::linprog(c, A, b, x);
    interior = x.head<2>(); 

    return minmaxsd < 0.0 && !std::isinf(minmaxsd);
  }

  double Environment::findInteriorDist(const Eigen::MatrixXd &hPoly, Eigen::Vector2d &interior) {
    int m = hPoly.cols();

    Eigen::MatrixXd A(m, 3);
    Eigen::VectorXd b(m), c(3), x(3);
    A.leftCols<2>() = hPoly.topRows<2>().transpose();
    A.rightCols<1>().setConstant(1.0);
    b = hPoly.topRows<2>().cwiseProduct(hPoly.bottomRows<2>()).colwise().sum().transpose();
    c.setZero();
    c(2) = -1.0;

    double minmaxsd = sdlp::linprog(c, A, b, x);
    interior = x.head<2>();

    return -minmaxsd;
  }

  bool Environment::filterCorridor(std::vector<Eigen::MatrixXd>& hPolys) {
    // return false;
    bool ret = false;
    if (hPolys.size() <= 2) {
      return ret;
    }
    std::vector<Eigen::MatrixXd> ret_polys;
    Eigen::MatrixXd hPoly0 = hPolys[0];
    Eigen::MatrixXd curIH;
    Eigen::Vector2d interior;
    for (int i = 2; i < (int)hPolys.size(); i++) {
      curIH.resize(4, hPoly0.cols() + hPolys[i].cols());
      curIH << hPoly0, hPolys[i];
      if (findInteriorDist(curIH, interior) < 1.0) {
        ret_polys.push_back(hPoly0);
        hPoly0 = hPolys[i - 1];
      } else {
        ret = true;
      }
    }
    ret_polys.push_back(hPoly0);
    ret_polys.push_back(hPolys.back());
    hPolys = ret_polys;
    return ret;
  }

  // void Environment::generateSFC(const std::vector<Eigen::Vector2d>& path,
  //                  const double bbox_width,
  //                  std::vector<Eigen::MatrixXd>& hPolys,
  //                  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& keyPts,
  //                  std::vector<std::vector<math::Vec2d>>& poly_vertices_set
  //                  ) {
  //   assert(path.size() > 1);
  //   vec_Vec2f obs_pc;
  //   EllipsoidDecomp2D decomp_util;
  //   decomp_util.set_local_bbox(Eigen::Vector2d(bbox_width, bbox_width));


  //   // int maxWidth = bbox_width / mapPtr_->resolution;
  //   int maxWidth;

  //   vec_E<Polyhedron2D> decompPolys;

  //   int path_len = path.size();

  //   int idx = 0;
  //   keyPts.clear();
  //   obs_pc.push_back(Vec2f(60, 60));
  //   obs_pc.push_back(Vec2f(-60, 60));
  //   obs_pc.push_back(Vec2f(-60, -60));
  //   obs_pc.push_back(Vec2f(60, -60));
  //   for (const auto& polygon : polygons_) {
  //       const auto& points = polygon.points();
  //       int num_points = points.size();
  //       for (int j = 0; j < num_points; ++j) {
  //         obs_pc.push_back(Vec2f(points[j].x(), points[j].y()));
  //       }
  //   }
  //   while (idx < path_len - 1) {
  //     int next_idx = idx;
  //     // looking forward -> get a farest next_idx
  //     while (next_idx + 1 < path_len && checkRayValid(path[idx], path[next_idx + 1])) {
  //       next_idx++;
  //     }
  //     // avoid same position
  //     if(next_idx == (path_len - 1))
  //     {
  //       while((path[next_idx] - path[idx]).norm() < 1e-4 && next_idx > 0)
  //       {
  //         next_idx --;
  //       }
  //     }
  //     // generate corridor with idx and next_idx
  //     vec_Vec2f line;
  //     line.push_back(path[idx]);
  //     line.push_back(path[next_idx]);
  //     keyPts.emplace_back(path[idx], path[next_idx]);
  //     decomp_util.set_obs(obs_pc);
  //     decomp_util.dilate(line);
  //     Polyhedron2D poly = decomp_util.get_polyhedrons()[0];
  //     const auto vertices = cal_vertices(poly);
  //     std::vector<math::Vec2d> poly_vertices;
  //     for (size_t i = 0; i < vertices.size(); i++) {
  //         math::Vec2d vertice(vertices[i](0), vertices[i](1));
  //         poly_vertices.push_back(vertice);
  //     }
  //     decompPolys.push_back(poly);
  //     poly_vertices_set.push_back(poly_vertices);
  //     // find a farest idx in current corridor
  //     idx = next_idx;
  //     while (idx + 1 < path_len && decompPolys.back().inside(path[idx + 1])) {
  //       idx++;
  //     }
  //   }

  //   hPolys.clear();
  //   Eigen::MatrixXd current_poly;
  //   for (uint i = 0; i < decompPolys.size(); i++) {
  //     vec_E<Hyperplane2D> current_hyperplanes = decompPolys[i].hyperplanes();
  //     current_poly.resize(4, current_hyperplanes.size());
  //     for (uint j = 0; j < current_hyperplanes.size(); j++) {
  //       current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
  //       //outside
  //     }
  //     hPolys.push_back(current_poly);
  //   }
  //   filterCorridor(hPolys);
  //   // check again
  //   Eigen::MatrixXd curIH;
  //   Eigen::Vector2d interior;
  //   std::vector<int> inflate(hPolys.size(), 0);
  //   for (int i = 0; i < (int)hPolys.size(); i++) {     
  //     if (findInteriorDist(current_poly, interior) < 0.1) {
  //       inflate[i] = 1;
  //     } else {
  //       compressPoly(hPolys[i], 0.1);
  //     }
  //   }
  //   for (int i = 1; i < (int)hPolys.size(); i++) {
  //     curIH.resize(4, hPolys[i - 1].cols() + hPolys[i].cols());
  //     curIH << hPolys[i - 1], hPolys[i];
  //     if (!findInterior(curIH, interior)) {
  //       if (!inflate[i - 1]) {
  //         compressPoly(hPolys[i - 1], -0.1);
  //         inflate[i - 1] = 1;
  //       }
  //     } else {
  //       continue;
  //     }
  //     curIH << hPolys[i - 1], hPolys[i];
  //     if (!findInterior(curIH, interior)) {
  //       if (!inflate[i]) {
  //         compressPoly(hPolys[i], -0.1);
  //         inflate[i] = 1;
  //       }
  //     }
  //   }
  // }

  void Environment::generateSFC(const std::vector<Eigen::Vector2d>& path,
                    const double bbox_width,
                    // std::vector<Eigen::MatrixXd>& hPolys,
                    std::vector<std::vector<std::vector<double>>>& hyperparam_set,
                    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& keyPts,
                    std::vector<std::vector<math::Vec2d>>& poly_vertices_set
                    ) {
      assert(path.size() > 1);
      std::vector<Eigen::MatrixXd> hPolys;
      vec_Vec2f obs_pc;
      EllipsoidDecomp2D decomp_util;
      vec_E<Polyhedron2D> decompPolys;
      int path_len = 0;
      for (int i = 0; i < path.size(); i++) {
        if (path[i](0) == 0 && path[i](1) == 0) {
          path_len = i;
          break;
        }
      }
      path_len = path.size();
      keyPts.clear();
      obs_pc.push_back(Vec2f(60, 60));
      obs_pc.push_back(Vec2f(-60, 60));
      obs_pc.push_back(Vec2f(-60, -60));
      obs_pc.push_back(Vec2f(60, -60));

      for (const auto& polygon : polygons_) {
          const auto& points = polygon.points();
          int num_points = points.size();
          for (int j = 0; j < num_points; ++j) {
            Eigen::Vector2d start = {points[j].x(), points[j].y()};
            Eigen::Vector2d end = {points[(j+1)%num_points].x(), points[(j+1)%num_points].y()};
            std::vector<Eigen::Vector2d> interpolatedPoints = interpolatePoints(start, end, 20);
            for (int i = 0; i < interpolatedPoints.size(); i++) {
              obs_pc.push_back(Vec2f(interpolatedPoints[i](0), interpolatedPoints[i](1)));
            }
          }
      }

      for (int idx = 0; idx < path_len; ++idx) {
          if(idx == 0 || idx == path_len - 1) {
            decomp_util.set_local_bbox(Eigen::Vector2d(1.0, 1.0));
          }
          decomp_util.set_local_bbox(Eigen::Vector2d(bbox_width, bbox_width));
          // Generate corridor for each waypoint
          int next_idx = idx + 1; // Next waypoint
          if (path[idx](0) == path[next_idx](0) && path[idx](1) == path[next_idx](1) && !poly_vertices_set.empty()) {
            poly_vertices_set.push_back(poly_vertices_set.back());
            decompPolys.push_back(decompPolys.back());
            continue;
          }
          vec_Vec2f line;
          line.push_back(path[idx]);
          if (idx == path_len -1) {
            line.push_back({path[idx](0) + 0.1, path[idx](1) + 0.1});
          }
          else {
            line.push_back(path[next_idx]);
          }
          keyPts.emplace_back(path[idx], path[next_idx]);

          decomp_util.set_obs(obs_pc);
          decomp_util.dilate(line);

          Polyhedron2D poly = decomp_util.get_polyhedrons()[0];     
          const auto vertices = cal_vertices(poly);
          std::vector<math::Vec2d> poly_vertices;
          for (size_t i = 0; i < vertices.size(); i++) {
              math::Vec2d vertice(vertices[i](0), vertices[i](1));
              poly_vertices.push_back(vertice);
          }
          decompPolys.push_back(poly);
          poly_vertices_set.push_back(poly_vertices);
      }

      hPolys.clear();
      Eigen::MatrixXd current_poly;
      for (uint i = 0; i < decompPolys.size(); i++) {
          vec_E<Hyperplane2D> current_hyperplanes = decompPolys[i].hyperplanes();
          current_poly.resize(4, current_hyperplanes.size());
          for (uint j = 0; j < current_hyperplanes.size(); j++) {
              current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
          }
          hPolys.push_back(current_poly);
      }

      // filterCorridor(hPolys);

      // Check and adjust corridors
      // Eigen::MatrixXd curIH;
      // Eigen::Vector2d interior;
      // std::vector<int> inflate(hPolys.size(), 0);
      // for (int i = 0; i < (int)hPolys.size(); i++) {     
      //     if (findInteriorDist(hPolys[i], interior) < 0.1) {
      //         inflate[i] = 1;
      //     } else {
      //         compressPoly(hPolys[i], 0.1);
      //     }
      // }
      // for (int i = 1; i < (int)hPolys.size(); i++) {
      //     curIH.resize(4, hPolys[i - 1].cols() + hPolys[i].cols());
      //     curIH << hPolys[i - 1], hPolys[i];
      //     if (!findInterior(curIH, interior)) {
      //         if (!inflate[i - 1]) {
      //             compressPoly(hPolys[i - 1], -0.1);
      //             inflate[i - 1] = 1;
      //         }
      //     } else {
      //         continue;
      //     }
      //     curIH << hPolys[i - 1], hPolys[i];
      //     if (!findInterior(curIH, interior)) {
      //         if (!inflate[i]) {
      //             compressPoly(hPolys[i], -0.1);
      //             inflate[i] = 1;
      //         }
      //     }
      // }
      for (int i = 0; i < hPolys.size(); i++) {
        std::vector<std::vector<double>> hyperparam;
        std::vector<double> test_co;
        for (int j = 0; j < hPolys[i].cols(); j++) {
          Eigen::Vector2d a = hPolys[i].col(j).head(2); 
          Eigen::Vector2d p = hPolys[i].col(j).tail(2);     
          double b = a.dot(p);   
          hyperparam.push_back({a(0), a(1), b});      
          test_co.push_back(a(0) * path[i](0) + a(1) * path[i](1) - b); 
        }
        hyperparam_set.push_back(hyperparam);
      }
  }

  void Environment::visCorridor(const vec_E<Polyhedron2D>& polyhedra, const ros::Publisher& hPolyPub) {
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = "odom";
    poly_msg.header.stamp = ros::Time::now();
    hPolyPub.publish(poly_msg);
  }
  
  void Environment::visCorridor(const std::vector<Eigen::MatrixXd>& hPolys, const ros::Publisher& hPolyPub) {
    vec_E<Polyhedron2D> decompPolys;
    for (const auto& poly : hPolys) {
      vec_E<Hyperplane2D> hyper_planes;
      hyper_planes.resize(poly.cols());
      for (int i = 0; i < poly.cols(); ++i) {
        hyper_planes[i].n_ = poly.col(i).head(2);
        hyper_planes[i].p_ = poly.col(i).tail(2);
      }
      decompPolys.emplace_back(hyper_planes);
    }
    visCorridor(decompPolys, hPolyPub);
  }

  void Environment::hPolys2Polys(const std::vector<Eigen::MatrixXd>& hPolys, std::vector<std::vector<Eigen::Vector2d>>& polygons) {
    for (const auto& poly : hPolys) {
      std::vector<Eigen::Vector2d> polygon; 
      for (int i = 0; i < poly.cols(); ++i) {
        Eigen::Vector2d n = poly.col(i).head<2>(); 
        double p = poly.col(i)(2);                 

        Eigen::Vector2d v = n * (-p / n.norm()); 
        polygon.push_back(v);
      }
      if (!polygon.empty()) {
        polygons.push_back(polygon);
      }
    }
  }

  std::vector<Eigen::Vector2d> Environment::interpolatePoints(const Eigen::Vector2d& start, const Eigen::Vector2d& end, int num_points) {
      std::vector<Eigen::Vector2d> points;
      points.reserve(num_points + 2); 

      Eigen::Vector2d step = (end - start) / (num_points + 1);

      for (int i = 0; i <= num_points + 1; ++i) {
          points.push_back(start + i * step);
      }

      return points;
  }

  bool Environment::visible_regions_feasible(const int& ind, const std::vector<double>& dis_scaler, const std::vector<Eigen::Vector2d>& intervals, const std::vector<Eigen::Vector2d>& points, int& tune_index){
    if (hypot(points[(ind + 1)%points.size()](0) - points[ind](0), points[(ind + 1)%points.size()](1) - points[ind](1)) < 2.5) {
      double interval_crr = intervals[ind](1) - intervals[ind](0);
      double interval_next = intervals[(ind + 1)%points.size()](1) - intervals[(ind + 1)%points.size()](0);
      if (dis_scaler[ind] == dis_scaler[(ind + 1)%points.size()]) {
        if (interval_crr >= interval_next) {
          tune_index = (ind + 1) % points.size();
        }
        else {
          tune_index = ind;
        }
      }
      else if (dis_scaler[ind] > dis_scaler[(ind + 1)%points.size()]) {
        tune_index = ind;
      }
      else {
        tune_index = (ind + 1) % points.size();
      }
      return false;
    }
    else {
      return true;
    }
  }

}