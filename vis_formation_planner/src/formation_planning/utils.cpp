#include "vis_formation_planner/utils.h"
#include <octomap/OcTree.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <to_solver/corridor.hpp>

using namespace std;
vector<vis_formation_planner::visualization::Color> colors = {
  visualization::Color::Purple,
  visualization::Color::Orange,
  visualization::Color::Blue,
  visualization::Color::Cyan,
};

void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<vis_formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for(int i = 0; i < 1e3; i++) {
    visualization::Delete(i, robot_name);
  }
  visualization::Trigger();

  nav_msgs::Path msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < sol_traj.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sol_traj.states[i].x;
    pose.pose.position.y = sol_traj.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
    msg.poses.push_back(pose);

    auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
    auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
    color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

void DrawCorasePathRivz(const std::vector<Eigen::Vector3d> path, int robot_index) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < path.size(); i++) {
    xs.push_back(path[i][0]);
    ys.push_back(path[i][1]);
  }
  visualization::Plot(xs, ys, 0.2, colors[robot_index], robot_index, robot_name);
  visualization::Trigger();
}

void DrawFGTrajRivz(const std::vector<vector<double>> traj, int robot_index, int traj_length) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < traj_length; i++) {
    xs.push_back(traj[i][0]);
    ys.push_back(traj[i][1]);
  }
  visualization::Plot(xs, ys, 0.5, colors[robot_index], robot_index, robot_name);
  visualization::Trigger();
}

// void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<vis_formation_planner::PlannerConfig> config,
//       int robot_index, ros::Publisher path_pub, double inflat) {
//   for (int i = 0; i < sol_traj.states.size(); i++) {
//     auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, inflat);
//     std::vector<double> x1, y1;
//     x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
//     y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
//     auto color = visualization::Color::White;
//     color.set_alpha(0.1);
//     visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
//   }
// //   const std::string robot_name = "Footprint_" + std::to_string(robot_index);
// //   for(int i = 0; i < 1e3; i++) {
// //     visualization::Delete(i, robot_name);
// //   }
// //   visualization::Trigger();

// //   nav_msgs::Path msg;
// //   msg.header.frame_id = "map";
// //   msg.header.stamp = ros::Time::now();
// //   for(size_t i = 0; i < sol_traj.states.size(); i++) {
// //     geometry_msgs::PoseStamped pose;
// //     pose.header = msg.header;
// //     pose.pose.position.x = sol_traj.states[i].x;
// //     pose.pose.position.y = sol_traj.states[i].y;
// //     pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
// //     msg.poses.push_back(pose);

// //     auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
// //     auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
// //     color.set_alpha(0.4);
// //     visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
// //   }
// //   path_pub.publish(msg);
//   visualization::Trigger();
// }

FullStates traj2fullstates(Trajectory_temp traj) {
    FullStates solution;
    solution.states.resize(traj.size());
    solution.tf = traj[traj.size() - 1].t;
    for (int i = 0; i < traj.size(); i++) {
        solution.states[i].x = traj[i].x;
        solution.states[i].y = traj[i].y;
        solution.states[i].theta = traj[i].theta;
        solution.states[i].v = traj[i].v;
        solution.states[i].phi = traj[i].phi;
        solution.states[i].a = traj[i].a;
        solution.states[i].omega = traj[i].omega;
    }
    return solution;
}

std::vector<std::vector<std::vector<double>>> InsertPath(const std::vector<std::vector<double>>& human_traj, int qn, int M, std::vector<std::vector<Eigen::Vector3d>> initTraj, const int& discrete) {
  int i, j, k;
  int N = M * discrete + 1;
  double dx;
  double dy;
  double dis;
  std::vector<std::vector<std::vector<double>>> threeDArray(qn, std::vector<std::vector<double>>(M, std::vector<double>(3)));

  for (i = 0; i < qn; i++){
    for(j = 0; j < M; j++){

        dx = (initTraj[i][j+1][0] - initTraj[i][j][0]) / discrete;
        dy = (initTraj[i][j+1][1] - initTraj[i][j][1]) / discrete;
        dis = (discrete * discrete) * (dx * dx + dy * dy); 
        for (k = 0; k < discrete; k++){
            plan[i][discrete*j+k][0] = initTraj[i][j][0] + dx*k;
            plan[i][discrete*j+k][1] = initTraj[i][j][1] + dy*k;
            // double rx = human_traj[discrete*j+k][0] - initTraj[i][j][0] + dx*k;
            // double ry = human_traj[discrete*j+k][1] - initTraj[i][j][1] + dy*k;
            // plan[i][discrete*j+k][5] = atan2(ry, rx);
            plan[i][discrete*j+k][5] = 0.0;

            if (dis < 0.1)
            {
              plan[i][discrete*j+k][3] = 0;
            }
            if (dis < 1.1){
              plan[i][discrete*j+k][3]=MAXV*0.7;
            }
            else{
              plan[i][discrete*j+k][3]=MAXV;
            }
        }
    }
    plan[i][discrete*M][0] = initTraj[i][M][0];
    plan[i][discrete*M][1] = initTraj[i][M][1];
  }
  for (i = 0; i < qn; i++){
    for(j = 0; j < M; j++){
        double dyaw = (plan[i][j+1][5] - plan[i][j][5]) / discrete;
        for (k = 0; k < discrete; k++){
          plan[i][discrete*j+k][6] = min(double(MAXGIMBLE), max(double(-MAXGIMBLE), dyaw));;
        }
    }
    plan[i][discrete*M][0] = initTraj[i][M][0];
    plan[i][discrete*M][1] = initTraj[i][M][1];
  }
  N = M*discrete+1;
  // 配准朝向角
  for (int qi = 0; qi < qn; qi++)
  {
    for (int next = discrete; next < N; next += discrete){
      if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
        continue;
      else{
        plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
        break;
      }
    }
  }
  double angle;
  for (i = 0; i < qn; i++){
    for(j = 0; j < M; j++){

      dx = initTraj[i][j+1][0] - initTraj[i][j][0];
      dy = initTraj[i][j+1][1] - initTraj[i][j][1];
      dis =  dx * dx + dy * dy;
      
      if (j > 0){
        if (dis > 0.1){
          angle = atan2(dy, dx);
          if (angle - plan[i][discrete*(j-1)][2] > M_PI)
            angle = angle - 2*M_PI;
          else if(plan[i][discrete*(j-1)][2] - angle > M_PI)
            angle = angle + 2*M_PI;
        }
          else angle = plan[i][discrete*(j-1)][2];
      }
      else {angle = plan[i][0][2];}

      for (k = 0; k < discrete; k++){
        plan[i][discrete*j+k][2] = angle;
      } 
    }
  }
  return plan;
}

void generateRegularPolygon(const double cx, const double cy, const double r, const int k, 
  std::vector<std::vector<double>>& vertice_initial_set) {
    std::vector<std::pair<double, double>> vertices;
    double angleIncrement = 2 * M_PI / k;

    for (int i = 0; i < k; ++i) {
        TrajectoryPoint tp_temp;
        double angle = i * angleIncrement;
        double x = cx + r * std::cos(angle);
        double y = cy + r * std::sin(angle);
        tp_temp.x = x;
        tp_temp.y = y;
        tp_temp.theta = 0.0;
        vertice_initial_set.push_back({x, y});
    }
}

void PlotPose(std::shared_ptr<vis_formation_planner::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type, const double& xo, const double& yo) {
    std::vector<double> xo_set, yo_set;
    std::vector<double> xo_, yo_;
    if (time_step > traj.size() - 1) {
        time_step = traj.size() - 1;
    }
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = traj[time_step].theta;
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(traj[time_step].x, traj[time_step].y, traj[time_step].theta);
    xo_set.push_back(robot_pos.x());
    xo_set.push_back(xo);
    xo_set.push_back(robot_pos.x());
    yo_set.push_back(robot_pos.y());
    yo_set.push_back(yo);
    yo_set.push_back(robot_pos.y());
    visualization::Plot(xo_set,yo_set, 0.5, visualization::Color::Green, 1, "cable" + std::to_string(robot_index));
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Red : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    xo_.push_back(xo+0.3); xo_.push_back(xo+0.3); xo_.push_back(xo-0.3); xo_.push_back(xo-0.3); xo_.push_back(xo+0.3);
    yo_.push_back(yo+0.3); yo_.push_back(yo-0.3); yo_.push_back(yo-0.3); yo_.push_back(yo+0.3); yo_.push_back(yo+0.3);
    visualization::Plot(xo_, yo_, 0.5, visualization::Color::Grey, 1, "point_s");
    visualization::Trigger();
    visualization::PlotPolygon(math::Polygon2d(box), 0.5, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

double getRandomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

// 生成在[-pi, pi]范围内的随机角度
double getRandomAngle() {
    return getRandomDouble(0, M_PI / 2);
}

TrajectoryPoint generateRandomnObstacle() {
  TrajectoryPoint point;
  // do
  {
    point.x = getRandomDouble(0, 50);
    point.y = getRandomDouble(0, 50);
    point.theta = getRandomAngle();
  } 
  // while (hypot(point.x, point.y) < 3 || hypot(point.x, point.y) > 5 );
  return point;
}

/*随机生成障碍物*/
void GenerateRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, std::vector<std::vector<double>>& obstacle,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat, std::vector<math::Polygon2d>& polys_inflat_ecbs,
std::vector<math::Polygon2d>& polys_inflat_ecbs_exe,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
) {
  height_set.clear();
  obstacle.clear();
  height.clear();
  polys.clear();
  polys_inflat.clear();
  poly_vertices_set.clear();
  for (int i = 0; i < num_obs; i++) {
    TrajectoryPoint obs_pt = generateRandomnObstacle();
    obstacle.push_back({obs_pt.x, obs_pt.y, obs_pt.theta});
  }
  // std::vector<int> obs_type_set = {1, 1, 3, 3, 6, 6};
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices, poly_vertices_ecbs, poly_vertices_ecbs_exe, poly_vertices_sfc;
    math::Vec2d centre(obstacle[obs_ind][0], obstacle[obs_ind][1]);
    // if (obs_ind < num_obs / 2) {
    poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.0, 0);
    poly_vertices_ecbs = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.7, 0);
    poly_vertices_ecbs_exe = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.5, 0);
    poly_vertices_sfc = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.55, 0);
    // }
    // else {
      // poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true, 0);
    // }
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
    polys_inflat.push_back(math::Polygon2d(poly_vertices_sfc));
    polys_inflat_ecbs.push_back(math::Polygon2d(poly_vertices_ecbs));
    polys_inflat_ecbs_exe.push_back(math::Polygon2d(poly_vertices_ecbs_exe));
  }
  }

bool ReadRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, const std::string& file_path,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat, std::vector<math::Polygon2d>& polys_inflat_ecbs,
std::vector<math::Polygon2d>& polys_inflat_ecbs_exe,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
) {
  std::ifstream file(file_path);
  if (!file.good()) {
    return false;
  }
  
  height_set.clear();
  height.clear();
  polys.clear();
  polys_inflat.clear();
  poly_vertices_set.clear();
  polys_inflat_ecbs.clear();
  polys_inflat_ecbs_exe.clear();
  std::vector<std::vector<double>> obstacle = readVectorsFromYAML(file_path);
  
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices, poly_vertices_ecbs, poly_vertices_ecbs_exe, poly_vertices_sfc;
    math::Vec2d centre(obstacle[obs_ind][0], obstacle[obs_ind][1]);
    
    poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.0, 0);
    poly_vertices_ecbs = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.5, 0);
    poly_vertices_ecbs_exe = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.5, 0);
    poly_vertices_sfc = generateobs.rotatePoint(centre, obstacle[obs_ind][2], 0.25, 0);
    
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
    polys_inflat.push_back(math::Polygon2d(poly_vertices_sfc));
    polys_inflat_ecbs.push_back(math::Polygon2d(poly_vertices_ecbs));
    polys_inflat_ecbs_exe.push_back(math::Polygon2d(poly_vertices_ecbs_exe));
  }
  return true;
}

  // Normalize angle to [0, 2π]
double normalizeTo2Pi(double angle) {
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}

// Normalize all angle ranges to [0, 2π]
std::vector<std::vector<std::pair<double, double>>> normalizeAngleRanges(
    const std::vector<std::vector<std::pair<double, double>>>& angleRanges) {
    std::vector<std::vector<std::pair<double, double>>> normalizedRanges;

    for (const auto& robotRanges : angleRanges) {
        std::vector<std::pair<double, double>> normalizedRobotRanges;

        for (const auto& range : robotRanges) {
            double start = normalizeTo2Pi(range.first);
            double end = normalizeTo2Pi(range.second);

            // Check if the range crosses the 0 boundary
            if (start > end) {
                // Split into two ranges: [start, 2π) and [0, end]
                normalizedRobotRanges.emplace_back(start, 2 * M_PI);
                normalizedRobotRanges.emplace_back(0.0, end);
            } else {
                // Normal range within [0, 2π)
                normalizedRobotRanges.emplace_back(start, end);
            }
        }

        normalizedRanges.push_back(normalizedRobotRanges);
    }

    return normalizedRanges;
}

// Function to calculate the endpoint (x, y) given distance and angle
Eigen::Vector2d calculateEndpoint(const Eigen::Vector2d& center, double distance, double angle) {
    double x = center(0) + distance * std::cos(angle);
    double y = center(1) + distance * std::sin(angle);
    return {x, y};
}

// Function to create and publish points as red markers in RViz
void drawPoints(const std::vector<Eigen::Vector2d>& points,
                const std::string& frame_id,
                const std::string& namespace_id,
                const int& id,
                ros::Publisher& marker_pub) {
    // Create a marker message
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = frame_id;  // Coordinate frame
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = namespace_id;          // Namespace for markers
    points_marker.id = id;                     // Unique ID for this marker
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;

    // Set scale (affects point size)
    points_marker.scale.x = 0.2;  // Point width
    points_marker.scale.y = 0.2;  // Point height

    // Set color (red)
    points_marker.color.r = 1.0;
    points_marker.color.g = 0.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;  // Alpha (opacity)

    // Calculate Cartesian coordinates for each point
    for (const auto& point : points) {
        auto x = point(0);
        auto y = point(1);
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0.0;  // Points lie on the XY plane
        points_marker.points.push_back(p);
    }

    // Publish the marker
    marker_pub.publish(points_marker);
}

std::vector<size_t> findIndices(const std::vector<math::Pose> traj) {
    vector<size_t> indices;
    if (traj.empty()) {
        return indices;
    }

    // Start with the first point
    indices.push_back(0);
    double accumulatedDistance = 0.0;

    for (size_t i = 1; i < traj.size(); ++i) {
        accumulatedDistance += hypot(traj[i-1].x() - traj[i].x(), traj[i-1].y() - traj[i].y());
        // Check if the accumulated distance is greater than 2m
        if (accumulatedDistance > 4) {
          indices.push_back(i);
          accumulatedDistance = 0.0; // Reset accumulated distance
        }
    }

    // Ensure the last point is included
    if (indices.back() != traj.size() - 1) {
        indices.push_back(traj.size() - 1);
    }

    return indices;
}

void calTimeAndMapping(const std::vector<std::pair<int, double>>& st_time_cons, std::vector<double>& time_profile, std::vector<int>& ref_index) {
  for (int i = 0; i < st_time_cons.size() - 1; i++) {
    for (int j = 0; j < st_time_cons[i+1].first - st_time_cons[i].first; j++) {
      double dt = (st_time_cons[i+1].second - st_time_cons[i].second) / (1.0 * (st_time_cons[i+1].first - st_time_cons[i].first));
      time_profile.push_back(dt);
      double time = st_time_cons[i].second + j * dt;
      int index = std::round(time * 10);
      ref_index.push_back(index);
    }
  }
}


int findClosestIndex(const std::vector<std::pair<int, double>>& vec, int index, int robot_ind, std::vector<std::vector<int>>& refined_index_set) {
    if (vec.empty() || index < 0 || index >= vec.back().first) {
        throw std::invalid_argument("Invalid index or empty vector.");
    }

    int closestIndex = -1;
    int closestStep = -1;
    int closestDistance = std::numeric_limits<int>::max();

    for (int i = 0; i < vec.size(); ++i) {
        if (std::find(refined_index_set[robot_ind].begin(), refined_index_set[robot_ind].end(), vec[i].first) != refined_index_set[robot_ind].end()) continue;
        int distance = std::abs(vec[i].first - index);
        if (distance < closestDistance || (distance == closestDistance && vec[i].first < vec[closestIndex].first)) {
            closestDistance = distance;
            closestIndex = i;
            closestStep = vec[i].first;
        }
    }
    refined_index_set[robot_ind].push_back(closestStep);
    return closestIndex;
}