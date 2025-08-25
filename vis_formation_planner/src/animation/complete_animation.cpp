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
#include "vis_formation_planner/math/generate_obs.h"
#include "vis_formation_planner/visualization/plot.h"
#include "vis_formation_planner/yaml_all.h"
#include "vis_formation_planner/visualization/matplotlibcpp.h"
#include <nav_msgs/Path.h>
#include "vis_formation_planner/utils.h"
#include "vis_formation_planner/visualization/visualization.h"

using namespace vis_formation_planner;
using namespace std;
namespace plt = matplotlibcpp;

void generate_sector(std::vector<double>& x, std::vector<double>& y, double radius, double angle, double start_angle = 0.0) {
    int num_points = 100;  
    double angle_step = angle / num_points;
    x.push_back(0);
    y.push_back(0);
    for (int i = 0; i <= num_points; ++i) {
        double theta = start_angle + i * angle_step + M_PI / 2;
        x.push_back(radius * std::cos(theta));
        y.push_back(radius * std::sin(theta));
    }
    x.push_back(0);
    y.push_back(0);
}

void generate_annulus(std::vector<double>& x, std::vector<double>& y, double radius, double angle, double start_angle = 0.0) {
    int num_points = 100;  
    double angle_step = angle / num_points;
    for (int i = 0; i <= num_points; ++i) {
        double theta = start_angle + i * angle_step + M_PI / 2;
        x.push_back(radius * std::cos(theta));
        y.push_back(radius * std::sin(theta));
    }
}

void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<vis_formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  for (int i = 0; i < sol_traj.states.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, 1.0);
    std::vector<double> x1, y1;
    x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    auto color = visualization::Color::White;
    color.set_alpha(0.03);
    visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
  }
  visualization::Trigger();
}

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

void PlotPoseFG(ros::Publisher& marker_pub, std::shared_ptr<vis_formation_planner::PlannerConfig> config, int robot_index, int time_step, vector<vector<vector<double>>> plan, vector<double> robot_type,  std::shared_ptr<visualize::visualize> visPtr) {
    std::vector<visualization::Color > color_set = {visualization::Color::Red, visualization::Color::Green, visualization::Color::Blue, visualization::Color::Purple};
    double theta = plan[robot_index][time_step][2];
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
    math::Pose robot_pos(plan[robot_index][time_step][0], plan[robot_index][time_step][1], theta);
    auto box = config->vehicle.GenerateBox(robot_pos);
    visPtr->publishFilledQuadrilateral(marker_pub, 
                            box.corners()[0].x(), box.corners()[0].y(), 
                            box.corners()[1].x(), box.corners()[1].y(), 
                            box.corners()[2].x(), box.corners()[2].y(),
                            box.corners()[3].x(), box.corners()[3].y(),
                            robot_index,
                            "robot");

}

void PlotCircle(const math::Pose& center, 
                double radius, 
                const visualization::Color& color, 
                const std::string& circle_name) {
    // 确定圆的分段数
    const int num_segments = 100;  // 圆的分段数量，越高越平滑
    std::vector<double> xs, ys;

    // 生成圆的点
    for (int i = 0; i < num_segments; ++i) {
        double theta = 2.0 * M_PI * i / num_segments;  // 当前角度
        double x = center.x() + radius * std::cos(theta);
        double y = center.y() + radius * std::sin(theta);
        xs.push_back(x);
        ys.push_back(y);
    }

    // 封闭圆形
    xs.push_back(xs.front());
    ys.push_back(ys.front());

    // 绘制圆
    visualization::Plot(xs, ys, radius, color, 1, circle_name);
    visualization::Trigger();
}

void PlotTrajectoryFG(std::shared_ptr<vis_formation_planner::PlannerConfig> config, const std::vector<std::vector<std::vector<double>>>& traj_set) {
    for (int i = 0; i < traj_set.size(); i++) {
        const std::string robot_name = "FGTraj_" + std::to_string(i);
        for (int j = 0; j < traj_set[i].size(); j++) {
            math::Pose robot_pos(traj_set[i][j][0], traj_set[i][j][1],traj_set[i][j][2]);
            auto box = config->vehicle.GenerateBox(robot_pos);
            auto color = visualization::Color::Yellow;
            color.set_alpha(0.03);
            // color.set_alpha(0.4);
            visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, j, robot_name);
            visualization::Trigger();           
        }
    }
}

void PlotPose(ros::Publisher& marker_pub, std::shared_ptr<vis_formation_planner::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type) {
    if (time_step > traj.size() - 1) {
        time_step = traj.size() - 1;
    }
    
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
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Green : visualization::Color::Cyan;
    visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, 1, "pose" + std::to_string(robot_index));
    visualization::Trigger();
}

void publishTriangle(ros::Publisher& marker_pub, double x, double y, double theta, double alpha, double L, int index) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom"; 
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle" + std::to_string(index);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST; 
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0; 
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.65; 
    marker.color.g = 0.16; 
    marker.color.b = 0.16; 
    marker.color.a = 0.2;  

    geometry_msgs::Point top, left, right;

    top.x = x;
    top.y = y;
    top.z = 0;

    left.x = x + L * cos(theta - alpha / 2.0);
    left.y = y + L * sin(theta - alpha / 2.0);
    left.z = 0;

    right.x = x + L * cos(theta + alpha / 2.0);
    right.y = y + L * sin(theta + alpha / 2.0);
    right.z = 0;

    marker.points.push_back(top);
    marker.points.push_back(left);
    marker.points.push_back(right);

    marker_pub.publish(marker);
}

int main(int argc, char* argv[])
{
    std::shared_ptr<visualize::visualize> visPtr_;
    auto config_ = std::make_shared<PlannerConfig>();
    config_->vehicle.InitializeDiscs();
    auto env = std::make_shared<Environment>(config_);
    std::string tail = "1";
    int j = 1; // ho, wang ours zhou
    // auto plan =  generate_traj_fg("/home/weijian/VisFormation/src/vis_formation_planner/result/wang/3/fg_50" + std::to_string(j) + ".yaml", 3);
    // std::vector<std::vector<double>> human_traj = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/wang/3/human_traj_50" + std::to_string(j) + ".yaml");
    auto plan =  generate_traj_fg("/home/weijian/VisFormation/src/vis_formation_planner/result/zhou/3/fg_comp90"  + tail + ".yaml", 3);
    std::vector<std::vector<double>> human_traj = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/zhou/3/human_traj_comp90" + tail + ".yaml");
    std::vector<std::vector<double>> angle_set = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/angle_set.yaml");
    std::string obs_path = "/home/weijian/VisFormation/src/vis_formation_planner/result/obstacle_comp" + std::to_string(90) + tail + ".yaml";
    double traj_len = 0.0;
    for (int i = 0; i < plan.size(); i++) {
        for (int j = 1; j < plan[i].size(); j++) {
            traj_len += hypot(plan[i][j][0]-plan[i][j-1][0], plan[i][j][1]-plan[i][j-1][1]);
        }
    }
    traj_len /= 3;
    auto time_profile = readVectorFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/time_profile" + tail + ".yaml");
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh;
    visualization::Init(nh, "odom", "/liom_test_vis");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	math::GenerateObstacle generateobs;
    std::vector<math::Polygon2d> polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe;
    std::vector<std::vector<double>> height_set;
    std::vector<double> height;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    bool plot_traj = false;

    // 定义三角形参数
    double x, y;    // 顶点坐标
    double alpha = 100 * M_PI / 180;  
    double L = 5.0;             
    std::vector<ros::Publisher> path_pub_set;
    ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/liom_test_path_car1", 1, false);
    ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/liom_test_path_car2", 1, false);
    ros::Publisher path_pub_car3 = nh.advertise<nav_msgs::Path>("/liom_test_path_car3", 1, false);
    ros::Publisher path_pub_car4 = nh.advertise<nav_msgs::Path>("/liom_test_path_car4", 1, false);
    ros::Publisher path_pub_car5 = nh.advertise<nav_msgs::Path>("/liom_test_path_car5", 1, false);
    path_pub_set.push_back(path_pub_car1); path_pub_set.push_back(path_pub_car2); path_pub_set.push_back(path_pub_car3);
    path_pub_set.push_back(path_pub_car4); path_pub_set.push_back(path_pub_car5); 
    ReadRandomObstacle(50, height_set, obs_path, height, polys, polys_inflat, polys_inflat_ecbs, polys_inflat_ecbs_exe, poly_vertices_set, generateobs);
    env->polygons() = polys;
    // std::vector<double> theta_set_ = {1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, -0.78539816339744828,
    // -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828};
    // std::vector<double> theta_set = {-0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -1.5707963267948966, -0.78539816339744828,
    //   -1.5707963267948966, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -1.5707963267948966};
    std::vector<double> theta_set_;
    std::vector<double> theta_set;
    for (int i = 0; i < plan.size(); i++) {
        theta_set_.push_back(plan[i].front()[2]);
        // theta_set.push_back(plan[i].back()[2]);
    }
    theta_set.push_back(0.0); theta_set.push_back(0.0); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.78539816339744828); theta_set.push_back(1.5707963267948966);
    theta_set.push_back(0); theta_set.push_back(0.0); theta_set.push_back(0); theta_set.push_back(0.0); theta_set.push_back(0.0);
    theta_set.push_back(0.0); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.0); theta_set.push_back(0.0);
    theta_set.push_back(0.0); theta_set.push_back(1.5707963267948966); theta_set.push_back(1.5707963267948966);
    j = 0;
    vector<double> robot_type(3, 0);
    std::vector<std::vector<std::vector<double>>> fov_set(plan.size()), fov_invalid(plan.size());
    int pre_occ_index = 0;
    while (ros::ok()) {
        std::vector<double> xs, ys;
        for(auto &pose: human_traj) {
          xs.push_back(pose[0]); ys.push_back(pose[1]);
        }
        // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, 1, "Coarse Path");
        // // visualization::PlotDashed(xs, ys, 0.02, visualization::Color::Grey, 0, "Coarse Path", 0.2, 0.1);
        // visualization::Trigger();
        // PlotTrajectoryFG(config_, plan);
        // for (int i = 0; i < 4; i++) {
        // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
        // }
        for (int i = 0; i < poly_vertices_set.size(); i++) {
            visPtr_->publishFilledQuadrilateral(marker_pub, 
                                       poly_vertices_set[i][0].x(), poly_vertices_set[i][0].y(), 
                                       poly_vertices_set[i][1].x(), poly_vertices_set[i][1].y(), 
                                       poly_vertices_set[i][2].x(), poly_vertices_set[i][2].y(),
                                       poly_vertices_set[i][3].x(), poly_vertices_set[i][3].y(),
                                       i,
                                       "obstacle");
        }

        math::Pose pose(human_traj[j][0], human_traj[j][1], 0.0);
        math::Pose pose_1(plan[0][j][0], plan[0][j][1], 0.0);
        math::Pose pose_2(plan[1][j][0], plan[1][j][1], 0.0);
        math::Pose pose_3(plan[2][j][0], plan[2][j][1], 0.0);
        // math::Pose pose_4(plan[3][j][0], plan[3][j][1], 0.0);
        visualization::Color  color;
        color = visualization::Color::Magenta;
        PlotCircle(pose, 0.25, color, "human" + std::to_string(0));
        color = visualization::Color::Red;
        color.set_alpha(0.3);
        PlotCircle(pose_1, 0.25, color, "robot1" + std::to_string(j));
        color = visualization::Color::Green;
        color.set_alpha(0.3);
        PlotCircle(pose_2, 0.25, color, "robot2" + std::to_string(j));
        color = visualization::Color::Blue;
        color.set_alpha(0.3);
        PlotCircle(pose_3, 0.25, color, "robot3" + std::to_string(j));
        // color = visualization::Color::Purple;
        // color.set_alpha(0.3);
        // PlotCircle(pose_4, 0.25, color, "robot4" + std::to_string(j));
        visualization::Trigger();

		for (int i = 0; i < plan.size(); i++) {
            PlotPoseFG(marker_pub, config_, i, j, plan, robot_type, visPtr_);
            publishTriangle(marker_pub, plan[i][j][0], plan[i][j][1], plan[i][j][5], alpha, L, i);
            double x_vec = human_traj[j][0] - plan[i][j][0];
            double y_vec = human_traj[j][1] - plan[i][j][1];
            double theta = M_PI / 2 - plan[i][j][5];
            double x_vec_ = x_vec * cos(theta) - y_vec * sin(theta);
            double y_vec_ = x_vec * sin(theta) + y_vec * cos(theta);
            Eigen::Vector2d p = {plan[i][j][0], plan[i][j][1]};
            Eigen::Vector2d center = {human_traj[j][0], human_traj[j][1]};
            if (y_vec_ < 0 || hypot(x_vec_, y_vec_) > 5.9 ) {
                continue;
            }
            if (!env->checkRayValid(p, center)) {
                if (j - pre_occ_index > 25) {
                    ros::Rate rate(1);
                    rate.sleep();	
                }
                pre_occ_index = j;
            }
            if (env->checkRayValid(p, center) && atan2(y_vec_, x_vec_) < 5 * M_PI / 6 && atan2(y_vec_, x_vec_) > M_PI / 6) {
            // if (env->checkRayValid(p, center)) {
                fov_set[i].push_back({x_vec_, y_vec_});
            }
            else {
                fov_invalid[i].push_back({x_vec_, y_vec_});
                // continue;
            }
        }

        j++;
        if (j == plan[0].size() - 3) {
            break;
        }
        // for (int i = 0; i < plan.size(); i++) {
        //     DrawFGTrajRivz(plan[i], i, plan[0].size());
        // }
        // for (int n = 0; n < plan.size(); n++) {
        //     for (int o = 0; o < plan[n].size(); o++) {
        //         double x_vec = human_traj[o][0] - plan[n][o][0];
        //         double y_vec = human_traj[o][1] - plan[n][o][1];
        //         double theta = M_PI / 2 - plan[n][o][5];
        //         double x_vec_ = (x_vec * cos(theta) - y_vec * sin(theta));
        //         double y_vec_ = (x_vec * sin(theta) + y_vec * cos(theta));zhou
        //             color = visualization::Color::Green;
        //         }
        //         else {
        //             color = visualization::Color::Red;
        //         }
        //         // if (y_vec_ < 0) {
        //             // y_vec_ = -y_vec_;
        //         // }
        //         // || hypot(x_vec_, y_vec_) >= 6 || atan2(y_vec_, x_vec_) > 145.0 / 180.0 * M_PI || atan2(y_vec_, x_vec_) < 35.0 / 180.0  * M_PI
        //         if (hypot(x_vec_, y_vec_) <= 2 || atan2(y_vec_, x_vec_) > 130.0 / 180.0 * M_PI ||  atan2(y_vec_, x_vec_) < 50 / 180.0 * M_PI) {
        //         // if (hypot(x_vec_, y_vec_) <= 2) {
        //             continue;
        //         }
        //         // if (env->checkRayValid(p, center) && atan2(y_vec_, x_vec_) < 145.0 / 180.0 * M_PI && atan2(y_vec_, x_vec_) > 35.0 / 180.0  * M_PI) {
        //         if (env->checkRayValid(p, center)) {
        //             fov_set[n].push_back({x_vec_, y_vec_});
        //         }
        //         else {
        //             fov_invalid[n].push_back({x_vec_, y_vec_});
        //         }
        //         std::vector<double> xo_set, yo_set;
        //         xo_set = {human_traj[o][0], plan[n][o][0]};
        //         yo_set = {human_traj[o][1], plan[n][o][1]};
        //         color.set_alpha(0.2);
        //         visualization::Plot(xo_set,yo_set, 0.1, color, 1, "cable" + std::to_string(n) + std::to_string(o));
        //         visualization::Trigger();
        //     }
        // }
        // break;
        ros::spinOnce();
        ros::Rate rate(50);
        rate.sleep();	
	}
    std::vector<double> x3, y3;
    std::vector<double> x4, y4;
    for (int index = 0; index < 3; index++) {
        std::vector<double> x1, y1;
        generate_sector(x1, y1, 7.5, M_PI * 2.0 / 3.0, -M_PI / 3.0); 
        std::vector<double> x2, y2;
        generate_annulus(x2, y2, 3.0, M_PI * 2.0 / 3.0, -M_PI / 3.0);
        plt::plot(x1, y1, {{"color", "red"}});
        plt::plot(x2, y2, {{"color", "green"}, {"linestyle", "--"}});
        for (int i = 0; i < fov_set[index].size(); i++) {
            x3.push_back(fov_set[index][i][0]);
            y3.push_back(fov_set[index][i][1]);
        }
        for (int i = 0; i < fov_invalid[index].size(); i++) {
            x4.push_back(fov_invalid[index][i][0]);
            y4.push_back(fov_invalid[index][i][1]);
        }
    }
    std::vector<std::vector<double>> fov_set_, fov_invalid_;
    fov_set_.push_back(x3);
    fov_set_.push_back(y3);
    fov_invalid_.push_back(x4);
    fov_invalid_.push_back(y4);
    writeVectorsToYaml(fov_set_, "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/fov_valid.yaml");
    writeVectorsToYaml(fov_invalid_, "/home/weijian/VisFormation/src/vis_formation_planner/result/ours/fov_invalid.yaml");
    plt::scatter(x3, y3, 30.0, {{"marker", "*"}, {"color", "blue"}, {"label", "Visible target"}});
    std::cout << "Visibility Success rate: " << double(x3.size()) / (double(plan[0].size() * 3)) * 100 << "%" << std::endl;
    plt::scatter(x4, y4, 30.0, {{"marker", "*"}, {"color", "orange"}, {"label", "Invisible targert"}});
    // 设置字体大小
    plt::legend({{"fontsize", "12"}});           // 图例字体大小
    plt::xlabel("x (m)", {{"fontsize", "14"}});  // x轴标签字体
    plt::ylabel("y (m)", {{"fontsize", "14"}});  // y轴标签字体
    plt::title("The target projewangcted to the camera’s FOV.", {{"fontsize", "16"}});  // 标题字体
    plt::show();
    std::vector<std::vector<double>> angles_set(plan.size());
    for (int i = 0; i < plan.size(); i++) {
        for (int j = 0; j < plan[i].size(); j++) {
            std::vector<double> ego_diff(2);
            std::vector<double> neighbor_diff(2);
            ego_diff[0] = plan[i][j][0] - human_traj[j][0];
            ego_diff[1] = plan[i][j][1] - human_traj[j][1];
            neighbor_diff[0] = plan[(i + 1) % plan.size()][j][0] - human_traj[j][0];
            neighbor_diff[1] = plan[(i + 1) % plan.size()][j][1] - human_traj[j][1];
            //||p(tk) − qk||
            double norm_ego_diff = sqrt(ego_diff[0] * ego_diff[0] + ego_diff[1] * ego_diff[1]);
            //||pϕ(tk) − qk||
            double norm_neighbor_diff = sqrt(neighbor_diff[0] * neighbor_diff[0] + neighbor_diff[1] * neighbor_diff[1]);
            // (p(tk) − qk) · (pϕ(tk) − qk)
            double dot_product = ego_diff[0] * neighbor_diff[0] + ego_diff[1] * neighbor_diff[1];
            double dot_product_ = -ego_diff[0] * neighbor_diff[0] - ego_diff[1] * neighbor_diff[1];
            // ψs
            double psi_seperate = dot_product / (norm_ego_diff * norm_neighbor_diff);
            angles_set[i].push_back(psi_seperate);
        }
    }
    double target_theta = plan.size() == 2 ? M_PI / 2 : 2 * M_PI / plan.size();
    target_theta = cos(target_theta);
    double t = 0.0;
    std::vector<double> time_increament;
    std::vector<double> target_theta_set;
    time_increament.push_back(t);
    target_theta_set.push_back(target_theta);
    for (int i = 0; i < time_profile.size(); i++) {
        t += time_profile[i];
        time_increament.push_back(t);
        target_theta_set.push_back(target_theta);
    }
    plt::plot(time_increament, angles_set[0], {{"color", "red"}, {"label", "robot0"}});
    plt::plot(time_increament, angles_set[1], {{"color", "green"}, {"label", "robot1"}});
    plt::plot(time_increament, angles_set[2], {{"color", "blue"}, {"label", "robot2"}});
    plt::plot(time_increament, target_theta_set, {{"color", "black"}, {"label", "target seperate angle"}});
    plt::legend();
    // plt::xlabel("x(m)");
    plt::ylabel("t(s)");
    plt::title("The seperate angle for each robot.");
    plt::show();
	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
