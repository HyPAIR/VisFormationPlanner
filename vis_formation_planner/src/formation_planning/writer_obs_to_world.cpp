#include <iostream>
#include <fstream>
#include <string>
#include "vis_formation_planner/yaml_all.h"
#include "vis_formation_planner/optimizer_interface.h"
#include <vector>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

using namespace vis_formation_planner;

void generateRegularPolygon(const double r, const int k, 
  std::vector<std::vector<double>>& vertice_set) {
    double cx = 0.0, cy = 0.0;
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
        vertice_set.push_back({x, y});
    }
}

std::vector<double> ConvertEndpointsToPose(const ignition::math::Vector3d& start, const ignition::math::Vector3d& end)
{
    ignition::math::Vector3d center = (start + end) / 2.0;

    ignition::math::Vector3d direction = end - start;
    direction.Normalize();

    ignition::math::Quaterniond rotation;
    rotation.From2Axes(ignition::math::Vector3d::UnitZ, direction);

    ignition::math::Vector3d euler = rotation.Euler();

    std::vector<double> pose = {center.X(), center.Y(), center.Z(), euler.X(), euler.Y(), euler.Z()};
    return pose;
}
std::string generateCircles(double x_s, double y_s, double x_g, double y_g, double radius) {
    std::string link = R"(
    <model name="circles">
      <static>true</static>
      <link name="circle1_link">
        <pose>)" + std::to_string(x_s) + " " + std::to_string(y_s) + R"(0 0 0 0</pose> 
        <visual name="circle1_visual">
          <geometry>
            <cylinder>
              <radius>)" + std::to_string(radius) + R"(</radius> 
              <length>0.01</length> 
            </cylinder>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient> 
            <diffuse>1 1 0 1</diffuse> 
          </material>
        </visual>
      </link>

      <link name="circle2_link">
        <pose>)" + std::to_string(x_g) + " " + std::to_string(y_g) + R"(0 0 0 0</pose> 
        <visual name="circle2_visual">
          <geometry>
            <cylinder>
              <radius>)" + std::to_string(radius) + R"(</radius> 
              <length>0.01</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse> 
          </material>
        </visual>
      </link>

    </model>)";
    return link;
}
std::string generateLinks(int id, double x, double y, double z, double roll, double pitch, double yaw, double length) {
    std::string link = R"(
    <link name="line)"+ std::to_string(id) + R"(_link">
      <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + R"(</pose> 
      <visual name="line)"+ std::to_string(id) + R"(_visual">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>)" + std::to_string(length) + R"(</length>
          </cylinder>
        </geometry>
        <material>
        <ambient>1 0.75 0.8 1</ambient>
        <diffuse>1 0.75 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="line)"+ std::to_string(id) + R"(_collision">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>)" + std::to_string(length) + R"(</length>
          </cylinder>
        </geometry>
      </collision>
      <gravity>0</gravity>
    </link>)";
    return link;
}

// std::string generateRedObstacle(int id, double x, double y, double z, double roll, double pitch, double yaw, 
//     double obs_x, double obs_y, double height) {
//     std::string obstacle = R"(
//     <model name="red_obstacle_)"+ std::to_string(id) +R"(">
//       <static>true</static>
//       <link name="link">
//         <inertial>
//           <mass>1.0</mass>
//         </inertial>
//         <visual name="visual">
//           <geometry>
//             <box>
//               <size>)" + std::to_string(obs_x) + " " + std::to_string(obs_y) + " " + std::to_string(height) +R"(</size>
//             </box>
//           </geometry>
//           <material>
//             <ambient>1 0 0 1</ambient>
//             <diffuse>1 0 0 1</diffuse>
//           </material>
//         </visual>
//         <collision name="collision">
//           <geometry>
//             <box>
//               <size>)" + std::to_string(obs_x) + " " + std::to_string(obs_y) + " " + std::to_string(height) +R"(</size>
//             </box>
//           </geometry>
//         </collision>
//       </link>
//       <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + R"(</pose>
//     </model>)";
//     return obstacle;
// }

std::string generateRedObstacle(int id, double x, double y, double z, double roll, double pitch, double yaw, 
    double obs_x, double obs_y, double height) {
    std::string obstacle = R"(
    <model name="pink_obstacle_)"+ std::to_string(id) +R"(">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>)" + std::to_string(obs_x) + " " + std::to_string(obs_y) + " " + std::to_string(height) +R"(</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + R"(</pose>
    </model>)";
    return obstacle;
}

std::string generateWaypoints(std::vector<double> human_traj) {
    std::string waypoint = R"(
      <waypoint>)" + std::to_string(human_traj[0]) + " " + std::to_string(human_traj[1]) + R"(1.25</waypoint>)";
    return waypoint;
}

std::string generatePinkObstacle(int id, double x, double y, double z, double roll, double pitch, double yaw, 
    double obs_x, double obs_y, double height) {
    std::string obstacle = R"(
    <model name="pink_obstacle_)"+ std::to_string(id) +R"(">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>)" + std::to_string(obs_x) + " " + std::to_string(obs_y) + " " + std::to_string(height) +R"(</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0.75 0.8 1</ambient>
            <diffuse>1 0.75 0.8 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " " + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + R"(</pose>
    </model>)";
    return obstacle;
}


std::string generateThinCylinder(int id, double x, double y, double z, double radius, double height) {
    std::string cylinder = R"(
    <model name="cylinder_)"+ std::to_string(id) +R"(">
      <static>true</static>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>)" + std::to_string(radius) + R"(</radius>
              <length>)" + std::to_string(height) + R"(</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.65 0.16 0.16 1</ambient> <!-- 棕色 -->
            <diffuse>0.65 0.16 0.16 1</diffuse> <!-- 棕色 -->
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>)" + std::to_string(radius) + R"(</radius>
              <length>)" + std::to_string(height) + R"(</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
      <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + R"( 0 0 0</pose>
    </model>)";
    return cylinder;
}

std::string generateOrangeRectangle(int id, double x, double y, double z, double yaw, 
                                    double width, double height) {
    std::string rectangle = R"(
    <model name="orange_rectangle_)" + std::to_string(id) + R"(">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>)" + std::to_string(width) + " " + std::to_string(height) + R"( 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.5 0 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + " 0 0 " + std::to_string(yaw) + R"(</pose>
    </model>)";
    return rectangle;
}

int main() {
    VVCM vvcm;
    double z_init = 0;
    int num_obs = 30;
    double po_x = 0.0, po_y = 0.0, po_z = 0.0;
    double x_s = 0.0, y_s = 0.0, x_g = 0.0, y_g = 0.0;
    std::vector<std::vector<double>> pos_3d;
    std::vector<std::vector<double>> pos_2d; 
    std::vector<std::vector<int>> taut_set;
    std::vector<std::vector<double>> vertice_set;
    std::vector<std::vector<double>> robot_pos_set;
    generateRegularPolygon(6.55 / sqrt(3), num_robot, vertice_set);
    std::string file_front = "/home/weijian/VisFormation/src/vis_formation_planner/traj_result/flexible_formation/" + std::to_string(num_robot) + "/";
    std::string file_back = ".yaml";
    int index = 6;
    double obj_x = 0.0, obj_y = 0.0;
	  std::vector<Trajectory_temp> traj_set;
	  // traj_set.resize(num_robot);
    // for (int i = 0; i < num_robot; i++) {
    //     auto traj_temp = generate_traj(file_front + "traj_" + std::to_string(num_robot)+ std::to_string(i) + std::to_string(1000) + ".yaml");
    //     traj_set[i] = traj_temp; 
    //     obj_x += traj_temp[0].x;
    //     obj_y += traj_temp[0].y;
    // }
    // obj_x /= num_robot;
    // obj_y /= num_robot;
    // for (int j = 0; j < num_robot; j++) {
    //   robot_pos_set.push_back({traj_set[j][0].x, traj_set[j][0].y});
    //   x_s += traj_set[j][0].x;
    //   y_s += traj_set[j][0].y;
    //   x_g += traj_set[j][traj_set[j].size() - 1].x;
    //   y_g += traj_set[j][traj_set[j].size() - 1].y;
    // }
    // ForwardKinematics fk_test(num_robot, vvcm.zr, vertice_set, robot_pos_set);
    // fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, vvcm.zr);
    // for (int po_count = 0; po_count < pos_3d.size(); po_count++) {
    //     po_x += pos_3d[po_count][0];
    //     po_y += pos_3d[po_count][1];
    //     po_z += pos_3d[po_count][2];
    // }
    std::ofstream file("/home/weijian/VisFormation/src/gazebo_sfm_plugin/worlds/cafe3.world", std::ios::trunc);

    // 写入world文件的头部
    file << R"(
    <!-- Usage notes are located in this file, look for "Usage"-->
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <world name="default">
        <gui>
            <camera name="user_camera">
                <view_controller>orbit</view_controller>
                <pose>21.943390 -21.026598 40.442112 0.000000 0.835999 1.576000</pose>
            </camera>
            <plugin name="grid_plugin" filename="libgazebo_gui.so">
                <grid>false</grid>
                <origin>false</origin>
            </plugin>
        </gui>
        <light name="sun" type="directional">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>0 0.5 -0.9</direction>
        </light>

        <light name="point_light" type="point">
          <pose>0 -0.377195 6.59868 0 -0 0</pose>
          <diffuse>0.784314 0.784314 0.784314 1</diffuse>
          <specular>0.0980392 0.0980392 0.0980392 1</specular>
          <attenuation>
            <range>50</range>
            <constant>0.8</constant>
            <linear>0</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <cast_shadows>0</cast_shadows>
          <direction>0 0 -1</direction>
        </light>

        <include>
          <uri>model://ground_plane</uri>
        </include>
    )";

    // file << generateCircles(x_s / num_robot, y_s / num_robot, x_g / num_robot, y_g / num_robot, vvcm.formation_radius + 0.5) << std::endl;
    // for (int i = 0; i < num_robot; i++) {
    //     ignition::math::Vector3d start(traj_set[i][0].x, traj_set[i][0].y, 1.8);
    //     ignition::math::Vector3d end(po_x / pos_3d.size(), po_y / pos_3d.size(), po_z / pos_3d.size() + 0.95);
    //     // 计算姿态
    //     std::vector<double> pose = ConvertEndpointsToPose(start, end);
    //     double x = pose[0]; 
    //     double y = pose[1];
    //     double z = pose[2];
    //     z_init = end[2];
    //     double roll = pose[3];
    //     double pitch = pose[4];
    //     double yaw = pose[5];
    //     ignition::math::Vector3d dir = end - start;
    //     double length = dir.Length();
    //     file << generateLinks(i+1, x, y, z, roll, pitch, yaw, length) << std::endl;
    // }
    // file << R"(
    //   <plugin name="dynamic_lines_mover" filename="/home/weijian/VisFormation/devel/lib/libdynamic_lines_mover.so"/>
    // </model>)";
    double obs_x = 1.0;
    double obs_y = 1.0;
    std::vector<std::vector<double>> obs_list = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/obstacle_test90" + std::to_string(index) + ".yaml");
    std::vector<std::vector<double>> human_traj = readVectorsFromYAML("/home/weijian/VisFormation/src/vis_formation_planner/result/ours/4/human_traj_test90" + std::to_string(index) + ".yaml");
    // write obstacles
    for (int i = 0; i < obs_list.size(); i++) {
        double x = obs_list[i][0]; 
        double y = obs_list[i][1];
        double z = 0.0;
        double roll = 0;
        double pitch = 0;
        double yaw = obs_list[i][2];
        double height = 2.0;
        file << generateRedObstacle(i, x, y, z, roll, pitch, yaw, 1.0, 1.0, height) << std::endl;
        // if ((0 < i && i < 15) || (30 <= i && i < 65)) {
        //   file << generateRedObstacle(i, x, y, z, roll, pitch, yaw, 1.5, 0.75, height) << std::endl;
        // }
        // else if ((15 <= i && i < 30) || (65 <= i && i < 100)) {
        //   file << generatePinkObstacle(i, x, y, z, roll, pitch, yaw, 1.0, 0.5, height) << std::endl;
        // }
    }
    // for (int i = 0; i < traj_set.size(); ++i) {
    //   for (int j = 0; j < traj_set[0].size(); ++j) {
    //     double x = traj_set[i][j].x;
    //     double y = traj_set[i][j].y;
    //     double theta = traj_set[i][j].theta;  // Yaw angle for orientation
    //     double width = 0.5;  // Adjust width as needed
    //     double height = 0.3;  // Adjust height as needed
    //     file << generateOrangeRectangle(j + (i*traj_set[0].size() - 1), x, y, 0.01, theta, width, height) << std::endl;
    //   }
    // }
    // for (int i = 0; i < traj_set.size(); i++) {
    //   file << generateThinCylinder(i, traj_set[i][0].x, traj_set[i][0].y, 2, 0.05, 1.2) << std::endl;
    // }
    // write the blue ball
    file << R"(
    <actor name="actor1">
      <pose>0.0 0.0 1.25 0 0 0</pose>
      <skin>
        <filename>walk-green.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk-green.dae</filename>
        <scale>1.000000</scale>
        <interpolate_x>true</interpolate_x>
      </animation>

      <plugin name="actor1_plugin" filename="libPedestrianSFMPlugin.so">
        <velocity>0.3</velocity>
        <radius>0.4</radius>
        <animation_factor>5.1</animation_factor>
        <people_distance>6.0</people_distance>
        <!-- weights -->
        <goal_weight>2.0</goal_weight>
        <obstacle_weight>0.0</obstacle_weight> <!--10.0-->
        <social_weight>0.0</social_weight> <!--2.1-->
        <group_gaze_weight>0.0</group_gaze_weight>
        <group_coh_weight>0.0</group_coh_weight>
        <group_rep_weight>0.0</group_rep_weight>
        <ignore_obstacles>
          <model>cafe</model>
          <model>ground_plane</model>
        </ignore_obstacles>
        <trajectory>
          <cyclic>false</cyclic>)";
          for (int i = 0; i < human_traj.size(); i++) {
            file << generateWaypoints(human_traj[i]) << std::endl;
          }
          file << R"(
        </trajectory>
      </plugin>
    </actor>

  </world>
</sdf>)";

    file.close();
    std::cout << "World file generated successfully!" << std::endl;
    return 0;
}
