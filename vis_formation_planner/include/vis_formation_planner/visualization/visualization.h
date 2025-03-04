#pragma once
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>

namespace visualize {
using PublisherMap = std::unordered_map<std::string, ros::Publisher>;
enum Color { white,
             red,
             green,
             blue,
             yellow,
             greenblue };

class visualize {
 private:
  ros::NodeHandle nh_;
  PublisherMap publisher_map_;

  void setMarkerColor(visualization_msgs::Marker& marker,
                      Color color = blue,
                      double a = 1) {
    marker.color.a = a;
    switch (color) {
      case white:
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        break;
      case red:
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        break;
      case green:
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        break;
      case blue:
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        break;
      case yellow:
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
        break;
      case greenblue:
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 1;
        break;
    }
  }

  void setMarkerColor(visualization_msgs::Marker& marker,
                      double a,
                      double r,
                      double g,
                      double b) {
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
  }

  void setMarkerScale(visualization_msgs::Marker& marker,
                      const double& x,
                      const double& y,
                      const double& z) {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
  }

  void setMarkerPose(visualization_msgs::Marker& marker,
                     const double& x,
                     const double& y,
                     const double& z) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
  }
  template <class ROTATION>
  void setMarkerPose(visualization_msgs::Marker& marker,
                     const double& x,
                     const double& y,
                     const double& z,
                     const ROTATION& R) {
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    Eigen::Quaterniond r(R);
    marker.pose.orientation.w = r.w();
    marker.pose.orientation.x = r.x();
    marker.pose.orientation.y = r.y();
    marker.pose.orientation.z = r.z();
  }

 public:
  visualize(ros::NodeHandle& nh) : nh_(nh) {}

  template <class CENTER, class TOPIC>
  void visualize_a_ball(const CENTER& c,
                        const double& r,
                        const TOPIC& topic,
                        const Color color = blue,
                        const double a = 1) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerColor(marker, color, a);
    setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
    setMarkerPose(marker, c[0], c[1], c[2]);
    marker.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(marker);
  }

  template <class PC, class TOPIC>
  void visualize_pointcloud(const PC& pc, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
      publisher_map_[topic] = pub;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    sensor_msgs::PointCloud2 point_cloud_msg;
    point_cloud.reserve(pc.size());
    for (const auto& pt : pc) {
      point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
    }
    pcl::toROSMsg(point_cloud, point_cloud_msg);
    point_cloud_msg.header.frame_id = "world";
    point_cloud_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(point_cloud_msg);
  }

  template <class PATH, class TOPIC>
  void visualize_path(const PATH& path, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10);
      publisher_map_[topic] = pub;
    }
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped tmpPose;
    tmpPose.header.frame_id = "world";
    for (const auto& pt : path) {
      tmpPose.pose.position.x = pt[0];
      tmpPose.pose.position.y = pt[1];
      tmpPose.pose.position.z = pt[2];
      path_msg.poses.push_back(tmpPose);
    }
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    publisher_map_[topic].publish(path_msg);
  }

  template <class BALLS, class TOPIC>
  void visualize_balls(const BALLS& balls,
                       const TOPIC& topic,
                       const Color color = blue,
                       const double a = 0.2) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(balls.size() + 1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (const auto& ball : balls) {
      setMarkerPose(marker, ball[0], ball[1], ball[2]);
      auto d = 2 * ball.r;
      setMarkerScale(marker, d, d, d);
      marker_array.markers.push_back(marker);
      marker.id++;
    }
    publisher_map_[topic].publish(marker_array);
  }

  template <class ELLIPSOID, class TOPIC>
  void visualize_ellipsoids(const ELLIPSOID& ellipsoids,
                            const TOPIC& topic,
                            const Color color = blue,
                            const double a = 0.2) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    setMarkerColor(marker, color, a);
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(ellipsoids.size() + 1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (const auto& e : ellipsoids) {
      setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R);
      setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
      marker_array.markers.push_back(marker);
      marker.id++;
    }
    publisher_map_[topic].publish(marker_array);
  }

  template <class PAIRLINE, class TOPIC>
  // eg for PAIRLINE: std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>
  void visualize_pairline(const PAIRLINE& pairline, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    setMarkerPose(marker, 0, 0, 0);
    setMarkerColor(marker, yellow, 1);
    setMarkerScale(marker, 0.005, 0.005, 0.005);
    marker.points.resize(2 * pairline.size());
    for (size_t i = 0; i < pairline.size(); ++i) {
      marker.points[2 * i + 0].x = pairline[i].first[0];
      marker.points[2 * i + 0].y = pairline[i].first[1];
      marker.points[2 * i + 0].z = pairline[i].first[2];
      marker.points[2 * i + 1].x = pairline[i].second[0];
      marker.points[2 * i + 1].y = pairline[i].second[1];
      marker.points[2 * i + 1].z = pairline[i].second[2];
    }
    publisher_map_[topic].publish(marker);
  }

  template <class TOPIC>
  void visualize_arrow(const Eigen::Vector2d p0, const Eigen::Vector2d& p1, const TOPIC& topic, const Color& color = blue) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker arrow_msg;
    arrow_msg.type = visualization_msgs::Marker::ARROW;
    arrow_msg.action = visualization_msgs::Marker::ADD;
    arrow_msg.header.frame_id = "world";
    arrow_msg.id = 0;
    arrow_msg.points.resize(2);
    setMarkerPose(arrow_msg, 0, 0, 0);
    setMarkerScale(arrow_msg, 0.05, 0.1, 0);
    setMarkerColor(arrow_msg, color);
    arrow_msg.points[0].x = p0[0];
    arrow_msg.points[0].y = p0[1];
    arrow_msg.points[0].z = p0[2];
    arrow_msg.points[1].x = p1[0];
    arrow_msg.points[1].y = p1[1];
    arrow_msg.points[1].z = p1[2];
    publisher_map_[topic].publish(arrow_msg);
  }

  // v0 -> v1 theta
  template <class TOPIC>
  void visualize_fan_shape_meshes(const std::vector<Eigen::Vector2d>& v0,
                                  const std::vector<Eigen::Vector2d>& v1,
                                  const std::vector<double>& thetas,
                                  const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "world";
    marker.id = 0;
    setMarkerPose(marker, 0, 0, 0);
    setMarkerScale(marker, 1, 1, 1);
    setMarkerColor(marker, green, 0.1);
    int M = v0.size();
    // int M = 1;
    for (int i = 0; i < M; ++i) {
      Eigen::Vector2d dp = v1[i] - v0[i];
      double theta0 = atan2(dp.y(), dp.x());
      double r = dp.norm();
      geometry_msgs::Point center;
      center.x = v0[i].x();
      center.y = v0[i].y();
      center.z = v0[i].z();
      geometry_msgs::Point p = center;
      p.x += r * cos(theta0 - thetas[i]);
      p.y += r * sin(theta0 - thetas[i]);
      for (double theta = theta0 - thetas[i] + 0.1; theta < theta0 + thetas[i]; theta += 0.1) {
        marker.points.push_back(center);
        marker.points.push_back(p);
        p = center;
        p.x += r * cos(theta);
        p.y += r * sin(theta);
        marker.points.push_back(p);
      }
    }
    publisher_map_[topic].publish(marker);
  }

  template <class ARROWS, class TOPIC>
  // ARROWS: pair<Vector2d, Vector2d>
  void visualize_arrows(const ARROWS& arrows, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker clear_previous_msg;
    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::Marker arrow_msg;
    arrow_msg.type = visualization_msgs::Marker::ARROW;
    arrow_msg.action = visualization_msgs::Marker::ADD;
    arrow_msg.header.frame_id = "world";
    arrow_msg.id = 0;
    arrow_msg.points.resize(2);
    setMarkerPose(arrow_msg, 0, 0, 0);
    setMarkerScale(arrow_msg, 0.02, 0.05, 0);
    setMarkerColor(arrow_msg, yellow);
    visualization_msgs::MarkerArray arrow_list_msg;
    arrow_list_msg.markers.reserve(1 + arrows.size());
    arrow_list_msg.markers.push_back(clear_previous_msg);
    for (const auto& arrow : arrows) {
      arrow_msg.points[0].x = arrow.first[0];
      arrow_msg.points[0].y = arrow.first[1];
      arrow_msg.points[0].z = arrow.first[2];
      arrow_msg.points[1].x = arrow.second[0];
      arrow_msg.points[1].y = arrow.second[1];
      arrow_msg.points[1].z = arrow.second[2];
      arrow_list_msg.markers.push_back(arrow_msg);
      arrow_msg.id += 1;
    }
    publisher_map_[topic].publish(arrow_list_msg);
  }

  template <class TRAJ, class TOPIC>
  // TRAJ:
  void visualize_traj(const TRAJ& traj, const TOPIC& topic) {
    std::vector<Eigen::Vector2d> path;
    auto duration = traj.getTotalDuration();
    for (double t = 0; t < duration; t += 0.01) {
      path.push_back(traj.getPos(t));
    }
    visualize_path(path, topic);
    std::vector<Eigen::Vector2d> wayPts;
    for (const auto& piece : traj) {
      wayPts.push_back(piece.getPos(0));
    }
    visualize_pointcloud(wayPts, std::string(topic) + "_wayPts");
  }

  template <class TRAJLIST, class TOPIC>
  // TRAJLIST: std::vector<TRAJ>
  void visualize_traj_list(const TRAJLIST& traj_list, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker clear_previous_msg;
    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::Marker path_msg;
    path_msg.type = visualization_msgs::Marker::LINE_STRIP;
    path_msg.action = visualization_msgs::Marker::ADD;
    path_msg.header.frame_id = "world";
    path_msg.id = 0;
    setMarkerPose(path_msg, 0, 0, 0);
    setMarkerScale(path_msg, 0.02, 0.05, 0);
    visualization_msgs::MarkerArray path_list_msg;
    path_list_msg.markers.reserve(1 + traj_list.size());
    path_list_msg.markers.push_back(clear_previous_msg);
    double a_step = 0.8 / traj_list.size();
    double a = 0.1;
    geometry_msgs::Point p_msg;
    for (const auto& traj : traj_list) {
      setMarkerColor(path_msg, white, a);
      a = a + a_step;
      path_msg.points.clear();
      for (double t = 0; t < traj.getTotalDuration(); t += 0.01) {
        auto p = traj.getPos(t);
        p_msg.x = p.x();
        p_msg.y = p.y();
        p_msg.z = p.z();
        path_msg.points.push_back(p_msg);
      }
      path_list_msg.markers.push_back(path_msg);
      path_msg.id += 1;
    }
    publisher_map_[topic].publish(path_list_msg);
  }

  template <class PATHLIST, class TOPIC>
  // PATHLIST: std::vector<PATH>
  void visualize_path_list(const PATHLIST& path_list, const TOPIC& topic) {
    auto got = publisher_map_.find(topic);
    if (got == publisher_map_.end()) {
      ros::Publisher pub =
          nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      publisher_map_[topic] = pub;
    }
    visualization_msgs::Marker clear_previous_msg;
    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
    visualization_msgs::Marker path_msg;
    path_msg.type = visualization_msgs::Marker::LINE_STRIP;
    path_msg.action = visualization_msgs::Marker::ADD;
    path_msg.header.frame_id = "world";
    path_msg.id = 0;
    setMarkerPose(path_msg, 0, 0, 0);
    setMarkerScale(path_msg, 0.02, 0.05, 0);
    visualization_msgs::MarkerArray path_list_msg;
    path_list_msg.markers.reserve(1 + path_list.size());
    path_list_msg.markers.push_back(clear_previous_msg);
    setMarkerColor(path_msg, greenblue);
    for (const auto& path : path_list) {
      path_msg.points.resize(path.size());
      for (size_t i = 0; i < path.size(); ++i) {
        path_msg.points[i].x = path[i].x();
        path_msg.points[i].y = path[i].y();
        path_msg.points[i].z = path[i].z();
      }
      path_list_msg.markers.push_back(path_msg);
      path_msg.id += 1;
    }
    publisher_map_[topic].publish(path_list_msg);
  }
  void createRing(visualization_msgs::Marker& marker, double x_c, double y_c, double r_inner, double r_outer,
                  double theta_start, double theta_end, int num_points, int color) {
      std::vector<Color> colors = {red, green, blue, greenblue};
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::Marker::ADD;

      // Set marker color and scale
      marker.scale.x = 1.0; // Not used for TRIANGLE_LIST
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      setMarkerColor(marker, colors[color], 0.5);
      // Generate points for the ring
      double d_theta = (theta_end - theta_start) / num_points;

      for (int i = 0; i < num_points; ++i) {
          double theta1 = theta_start + i * d_theta;
          double theta2 = theta_start + (i + 1) * d_theta;

          // Points on the inner radius
          geometry_msgs::Point p1, p2;
          p1.x = x_c + r_inner * cos(theta1);
          p1.y = y_c + r_inner * sin(theta1);
          p1.z = 0.0;

          p2.x = x_c + r_inner * cos(theta2);
          p2.y = y_c + r_inner * sin(theta2);
          p2.z = 0.0;

          // Points on the outer radius
          geometry_msgs::Point p3, p4;
          p3.x = x_c + r_outer * cos(theta1);
          p3.y = y_c + r_outer * sin(theta1);
          p3.z = 0.0;

          p4.x = x_c + r_outer * cos(theta2);
          p4.y = y_c + r_outer * sin(theta2);
          p4.z = 0.0;

          // Add two triangles to form a quad
          marker.points.push_back(p1);
          marker.points.push_back(p3);
          marker.points.push_back(p4);

          marker.points.push_back(p1);
          marker.points.push_back(p4);
          marker.points.push_back(p2);
      }
  }
  void publishAnnulus(const int& ind, const std::vector<Eigen::Vector2d>& intervals, const double& x_c, const double& y_c, const std::vector<double>& dis_scaler, const int& r_inner, const int& r_outer, const ros::Publisher& marker_pub) {
    for (size_t i = 0; i < intervals.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = ros::Time::now();
      marker.ns = "ring" + std::to_string(ind);
      marker.id = i; 
      // const auto& interval = robotIntervals[i][selectedIntervals[i]];
      double theta_start = intervals[i](0);  
      double theta_end = intervals[i](1);
      createRing(marker, x_c, y_c, r_inner * dis_scaler[i], r_outer * dis_scaler[i], theta_start, theta_end, 50, i);
      marker_pub.publish(marker);
    }
  }

  void publishPolygon(const std::vector<std::vector<Eigen::Vector2d>>& polygons,
                      const std::string& frame_id,
                      const std::string& namespace_id,
                      const int& id,
                      const ros::Publisher& marker_pub) {

      // Marker message for lines
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = frame_id;
      line_marker.header.stamp = ros::Time::now();
      line_marker.ns = namespace_id;
      line_marker.id = 0;
      line_marker.type = visualization_msgs::Marker::LINE_LIST;
      line_marker.action = visualization_msgs::Marker::ADD;

      // Set line properties
      line_marker.scale.x = 0.05; // Line width
      line_marker.color.r = 1.0;  // Red color
      line_marker.color.g = 0.0;
      line_marker.color.b = 0.0;
      line_marker.color.a = 1.0;  // Fully opaque

      // Add lines for each polygon
      for (const auto& polygon : polygons) {
          for (size_t i = 0; i < polygon.size(); ++i) {
              geometry_msgs::Point p1, p2;

              // Current vertex
              p1.x = polygon[i].x();
              p1.y = polygon[i].y();
              p1.z = 0.0;

              // Next vertex (loop back to the first vertex for the last edge)
              p2.x = polygon[(i + 1) % polygon.size()].x();
              p2.y = polygon[(i + 1) % polygon.size()].y();
              p2.z = 0.0;

              // Add line segment (p1 -> p2)
              line_marker.points.push_back(p1);
              line_marker.points.push_back(p2);
          }
      }
      // Publish marker
      marker_pub.publish(line_marker);
  }
  void publishFilledQuadrilateral(ros::Publisher& marker_pub, 
                                  double x1, double y1, 
                                  double x2, double y2, 
                                  double x3, double y3, 
                                  double x4, double y4, 
                                  int index,
                                  const std::string ns) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom"; 
      marker.header.stamp = ros::Time::now();
      marker.ns = ns + std::to_string(index);
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

      if (ns == "obstacle") {
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0; 
          marker.color.a = 1.0; 
      }
      else {
          if (index == 0) {
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0; 
              marker.color.a = 1.0; 
          }
          else if (index == 1) {
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0; 
              marker.color.a = 1.0; 
          }
          else if (index == 2) {
              marker.color.r = 0.0;
              marker.color.g = 0.0;
              marker.color.b = 1.0; 
              marker.color.a = 1.0; 
          }
          else if (index == 3) {
              marker.color.r = 0.5;
              marker.color.g = 0.0;
              marker.color.b = 0.5; 
              marker.color.a = 1.0; 
          }
      }

      geometry_msgs::Point p1, p2, p3, p4;

      p1.x = x1; p1.y = y1; p1.z = 0;
      p2.x = x2; p2.y = y2; p2.z = 0;
      p3.x = x3; p3.y = y3; p3.z = 0;
      p4.x = x4; p4.y = y4; p4.z = 0;

      marker.points.push_back(p1);
      marker.points.push_back(p2);
      marker.points.push_back(p3);

      marker.points.push_back(p1);
      marker.points.push_back(p3);
      marker.points.push_back(p4);

      std_msgs::ColorRGBA vertex_color;
      if (ns == "obstacle") {
          vertex_color.r = 0.0;
          vertex_color.g = 0.0;
          vertex_color.b = 0.0; 
          vertex_color.a = 1.0; 
      }
      else {
          if (index == 0) {
              vertex_color.r = 1.0;
              vertex_color.g = 0.0;
              vertex_color.b = 0.0; 
              vertex_color.a = 1.0; 
          }
          else if (index == 1) {
              vertex_color.r = 0.0;
              vertex_color.g = 1.0;
              vertex_color.b = 0.0; 
              vertex_color.a = 1.0; 
          }
          else if (index == 2) {
              vertex_color.r = 0.0;
              vertex_color.g = 0.0;
              vertex_color.b = 1.0; 
              vertex_color.a = 1.0; 
          }
          else if (index == 3) {
              vertex_color.r = 0.5;
              vertex_color.g = 0.0;
              vertex_color.b = 0.5; 
              vertex_color.a = 1.0; 
          }
      }
      marker.colors.push_back(vertex_color);
      marker.colors.push_back(vertex_color);
      marker.colors.push_back(vertex_color);
      marker.colors.push_back(vertex_color);
      marker.colors.push_back(vertex_color);
      marker.colors.push_back(vertex_color);

      marker_pub.publish(marker);
  }
};

}  // namespace visualize
