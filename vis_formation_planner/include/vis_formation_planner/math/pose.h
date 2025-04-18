/**
 * file pose.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief define class Pose
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#pragma once
#include "vec2d.h"

namespace vis_formation_planner {
namespace math {

class Pose {
public:
  Pose() = default;
  Pose(double x, double y, double theta): x_(x), y_(y), theta_(theta) {}

  inline double x() const { return x_; }
  inline double y() const { return y_; }
  inline double theta() const { return theta_; }

  inline void setX(double x) { x_ = x; }
  inline void setY(double y) { y_ = y; }
  inline void setTheta(double theta) { theta_ = theta; }

  inline operator Vec2d() const { return { x_, y_}; }

  inline Pose relativeTo(const Pose &coord) const {
    double dx = x_ - coord.x();
    double dy = y_ - coord.y();
    return {
      dx * cos(coord.theta()) + dy * sin(coord.theta()),
      -dx * sin(coord.theta()) + dy * cos(coord.theta()),
      theta_ - coord.theta()
    };
  }

  inline Pose extend(double length) const {
    return transform({ length, 0, 0 });
  }

  inline Pose transform(const Pose &relative) const {
    return {
      x_ + relative.x() * cos(theta_) - relative.y() * sin(theta_),
      y_ + relative.x() * sin(theta_) + relative.y() * cos(theta_),
      theta_ + relative.theta_
    };
  }

  inline double DistanceTo(const Vec2d &rhs) const {
    return hypot(x() - rhs.x(), y() - rhs.y());
  }

  inline bool operator<(const Pose& other) const {
      return x_ < other.x();
  } 
  
  inline bool operator==(const Pose& other) const {
      return x_ == other.x() && y_ == other.y() && theta_ == other.theta();
  } 
private:
  double x_, y_, theta_;
};

}
}

