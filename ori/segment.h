
#pragma once

#include <list>
#include <cmath>
#include <cassert>
#include <iostream>

#include "point.h"

namespace obstacle_detector
{

class Segment
{
public:
  Segment(const Point& p1 = Point(), const Point& p2 = Point()) {
    //assert(p1 != p2);
    // Swap if not counter-clockwise.
    if (p1.cross(p2) > 0.0)
      p1_ = p1, p2_ = p2;
    else
      p1_ = p2, p2_ = p1;
  }

  void setFirstPoint(double x, double y) { p1_.x = x; p1_.y = y; }
  void setLastPoint(double x, double y) { p2_.x = x; p2_.y = y; }

  double length() const { return (p2_ - p1_).length(); }
  Point normal() const { return (p2_ - p1_).perpendicular().normalize(); }
  Point first_point() const { return p1_; }
  Point last_point() const { return p2_; }
  Point projection(const Point& p) const {
    Point a = p2_ - p1_;
    Point b = p - p1_;
    return p1_ + a.dot(b) * a / a.lengthSquared();     //p1_ + |a|*|b|*cos<a,b> * a/|a|*|a|,假设a、b角度为0， 最后得到的是拟合直线得到直线上的点
  }
  std::list<Point>& point_set() { return point_set_; }
  double distanceTo(const Point& p) const { return (p - projection(p)).length(); }   //p-长度？？

  friend std::ostream& operator<<(std::ostream& out, const Segment& s)
  { out << "p1: " << s.p1_ << ", p2: " << s.p2_; return out; }

private:
  Point p1_;
  Point p2_;
  std::list<Point> point_set_;
};

} // namespace obstacle_detector
