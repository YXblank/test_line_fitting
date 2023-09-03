
#pragma once

#define ARMA_DONT_USE_CXX11

#include <armadillo>
#include <cassert>
#include <list>

#include "point.h"
#include "segment.h"
#include "circle.h"

namespace obstacle_detector
{

/*
 * Returns a total best fit approximation of
 * segment based on given point set. The equation     返回基于给定点集的段的总最佳拟合近似值。用于拟合的方程如下所示：Ax + By = -C
 * used for fitting is given by                        对A，B，C参数进行了归一化处理。
 *    Ax + By = -C
 * and the A, B, C parameters are normalized.
 */
Segment fitSegment(const std::list<Point>& point_set) {
  int N = point_set.size();
  assert(N >= 2);                                 // 条件错误，结束运行

  arma::mat input  = arma::mat(N, 2).zeros();  // [x_i, y_i]   N×2的矩阵，元素全为0
  arma::vec output = arma::vec(N).ones();      // [-C]             元素全为1  N行 1列
  arma::vec params = arma::vec(2).zeros();     // [A ; B]          元素全为0  2行 1列   这里假设 input* params = output

  int i = 0;
  for (const Point& point : point_set) {
    input(i, 0) = point.x;
    input(i, 1) = point.y;               // 将point_set 点全部存到矩阵 input 中
    ++i;
  }

  // Find A and B coefficients from linear regression (assuming C = -1.0)   从线性回归中找出A和B系数（假设C=-1.0）
  params = arma::pinv(input) * output;    //求 input 的伪逆矩阵         pinv(input)×input* params = pinv(input)×output     

  double A, B, C;
  A = params(0);
  B = params(1);
  C = -1.0;

  // Find end points
  Point p1 = point_set.front();
  Point p2 = point_set.back();

  double D = (A * A + B * B);

  if (D > 0.0) {  // Project end points on the line
    Point projected_p1;
    projected_p1.x = ( B * B * p1.x - A * B * p1.y - A * C) / D;
    projected_p1.y = (-A * B * p1.x + A * A * p1.y - B * C) / D;

    Point projected_p2;
    projected_p2.x = ( B * B * p2.x - A * B * p2.y - A * C) / D;
    projected_p2.y = (-A * B * p2.x + A * A * p2.y - B * C) / D;

    return Segment(projected_p1, projected_p2);
  }

   return Segment(p1, p2);
}

/*
 * Returns a total best fit approximation of
 * a circle based on given point set. The equation
 * used for fitting is given by
 *   a1 * x + a2 * y + a3 = -(x^2 + y^2)
 * where parameters a1, a2, a3 are obtained from
 * circle equation
 *   (x-x0)^2 + (y-y0)^2 = r^2.
 */
Circle fitCircle(const std::list<Point>& point_set)
{
  int N = point_set.size();
  assert(N >= 3);

  arma::mat input  = arma::mat(N, 3).zeros();   // [x_i, y_i, 1]
  arma::vec output = arma::vec(N).zeros();      // [-(x_i^2 + y_i^2)]
  arma::vec params = arma::vec(3).zeros();      // [a_1 ; a_2 ; a_3]

  int i = 0;
  for (const Point& point : point_set) {
    input(i, 0) = point.x;
    input(i, 1) = point.y;
    input(i, 2) = 1.0;

    output(i) = -(pow(point.x, 2) + pow(point.y, 2));
    i++;
  }

  // Find a_1, a_2 and a_3 coefficients from linear regression
  params = arma::pinv(input) * output;

  Point center(-params(0) / 2.0, -params(1) / 2.0);
  double radius =  sqrt((params(0) * params(0) + params(1) * params(1)) / 4.0 - params(2));

  return Circle(center, radius);
}

} // namespace obstacle_detector
