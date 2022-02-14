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
  return util::round(sqrt((dx * dx) + (dy * dy)));
}

// Compute the difference of heights between two points
float Point::dist_z(const Point &other) const {
  return util::round(abs(other.z() - this->z()));
}

// Compute Euclidean distance between two points
float Point::dist(const Point &other) const {
  float dx = other.x() - this->x();
  float dy = other.y() - this->y();
  float dz = other.z() - this->z();
  return util::round(sqrt((dx * dx) + (dy * dy) + (dz * dz)));
}

// Compute the angle between two points
float Point::angle_xy(const Point &other) const {
  float dx = other.x() - this->x();
  float dy = other.y() - this->y();
  return util::round(atan2(dy, dx));
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
