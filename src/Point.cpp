/**
 * @file Point.cpp
 * @brief Source file for class Point
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Point.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Set all coordinates
void Point::set(float x, float y, float z) {
  this->x_ = x;
  this->y_ = y;
  this->z_ = z;
}

// Compute the Euclidean linear distance between two points
float Point::dist_xy(const Point &other) const {
  float dx = other.x() - this->x();
  float dy = other.y() - this->y();
  return nav::round(sqrt((dx * dx) + (dy * dy)));
}

// Compute the difference of heights between two points
float Point::dist_z(const Point &other) const {
  return nav::round(abs(other.z() - this->z()));
}

// Compute Euclidean distance between two points
float Point::dist(const Point &other) const {
  float dx = other.x() - this->x();
  float dy = other.y() - this->y();
  float dz = other.z() - this->z();
  return nav::round(sqrt((dx * dx) + (dy * dy) + (dz * dz)));
}

// Compute the angle between two points
float Point::angle_xy(const Point &other) const {
  float dx = other.x() - this->x();
  float dy = other.y() - this->y();
  return nav::round(atan2(dy, dx));
}

// Rotate point of theta on xy-axis
void Point::rotate_xy(const Point &cnt, float theta) {
  float xtemp = this->x_ - cnt.x();
  float ytemp = this->y_ - cnt.y();
  this->x_ =
      nav::round(((cos(theta) * xtemp) - (sin(theta) * ytemp)) + cnt.x());
  this->y_ =
      nav::round(((sin(theta) * xtemp) + (cos(theta) * ytemp)) + cnt.y());
}

// Check if the point is inside a polygon
bool Point::is_inside_xy(std::vector<Point> &points) {
  bool res = false;
  for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    if (((points[i].y() >= this->y_) != (points[j].y() >= this->y_)) &&
        (this->x_ <= (points[j].x() - points[i].x()) *
                             (this->y_ - points[i].y()) /
                             (points[j].y() - points[i].y()) +
                         points[i].x())) {
      res = !res;
    }
  }
  return res;
}

// Check if two points are equals
bool Point::operator==(const Point &other) {
  return ((this->x_ == other.x()) & (this->y_ == other.y()) &
          (this->z_ == other.z()));
}

// Convert a point in string form
std::ostream &operator<<(std::ostream &os, const Point &pnt) {
  os << "(" << pnt.x() << ", " << pnt.y() << ", " << pnt.z() << ")";
  return os;
}

} // namespace nav
