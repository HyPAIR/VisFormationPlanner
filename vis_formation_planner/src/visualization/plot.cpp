/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "vis_formation_planner/visualization/plot.h"

namespace vis_formation_planner {
namespace visualization {
namespace {
std::string frame_ = "map";

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 10, true);
}

void Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    // pt.z = 0.1 * id;
    pt.z = 0.0;
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void PlotDashed(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns, double dash_length = 0.1, double gap_length = 0.1) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns;
    msg.id = id >= 0 ? id : arr_.markers.size();
    
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_LIST;  // 使用 LINE_LIST 代替 LINE_STRIP
    msg.pose.orientation.w = 1.0;
    msg.scale.x = width;
    msg.color = color.toColorRGBA();

    // 计算每一段的总长度
    double total_length = 0.0;
    geometry_msgs::Point last_pt;
    bool has_last = false;

    for (size_t i = 1; i < xs.size(); i++) {
        geometry_msgs::Point pt1, pt2;
        pt1.x = xs[i - 1];
        pt1.y = ys[i - 1];
        pt1.z = 0.0;

        pt2.x = xs[i];
        pt2.y = ys[i];
        pt2.z = 0.0;

        double dx = pt2.x - pt1.x;
        double dy = pt2.y - pt1.y;
        double segment_length = std::sqrt(dx * dx + dy * dy);
        double remaining_length = segment_length;

        double ux = dx / segment_length;
        double uy = dy / segment_length;

        double current_pos = 0.0;
        bool draw = true;  // 控制绘制和空隙

        while (remaining_length > 0.0) {
            double step = draw ? dash_length : gap_length;
            if (remaining_length < step) {
                step = remaining_length;
            }

            geometry_msgs::Point start_pt, end_pt;
            start_pt.x = pt1.x + ux * current_pos;
            start_pt.y = pt1.y + uy * current_pos;
            start_pt.z = 0.0;

            end_pt.x = pt1.x + ux * (current_pos + step);
            end_pt.y = pt1.y + uy * (current_pos + step);
            end_pt.z = 0.0;

            if (draw) {
                msg.points.push_back(start_pt);
                msg.points.push_back(end_pt);
            }

            current_pos += step;
            remaining_length -= step;
            draw = !draw;  // 交替切换画线/空隙
        }
    }

    arr_.markers.push_back(msg);
}


void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  arr_.markers.push_back(msg);
}


void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(h, percent, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void PlotPoints(const Vector &xs, const Vector &ys, const std::vector<Color> &colors, double width, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(colors[i].toColorRGBA());
  }

  arr_.markers.push_back(msg);
}

void Delete(int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = ns;
  msg.id = id;

  msg.action = visualization_msgs::Marker::DELETE;
  arr_.markers.push_back(msg);
}

void Trigger() {
  publisher_.publish(arr_);
  arr_.markers.clear();
}

void Clear(const std::string &ns) {
  arr_.markers.clear();

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = ns;

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_.publish(arr);
}
}
}