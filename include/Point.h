/**
 * @file Point.h
 * @brief Header file for class Point
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef POINT_H
#define POINT_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <boost/serialization/set.hpp>
#include <cmath>
#include <iostream>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class Point {
private:
  float x_; // x-coordinate
  float y_; // y-coordinate
  float z_; // z-coordinate

  // Point serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &x_ &y_ &z_;
  }

public:
  // Default constructor
  Point() : x_(0.0f), y_(0.0f), z_(0.0f){};

  // Initialize a point
  Point(float x, float y, float z) : x_(x), y_(y), z_(z){};

  // Point copy-constructor
  Point(const Point &other) : x_(other.x()), y_(other.y()), z_(other.z()){};

  // Set x-coordinate
  void set_x(float x) { x_ = x; }
  // Get x-coordinate
  const float &x() const { return x_; }

  // Set y-coordinate
  void set_y(float y) { y_ = y; }
  // Get y-coordinate
  const float &y() const { return y_; }

  // Set z-coordinate
  void set_z(float z) { z_ = z; }
  // Get z-coordinate
  const float &z() const { return z_; }

  // Set all coordinates
  void set(float x, float y, float z);

  // Compute the Euclidean linear distance between two points
  float dist_xy(const Point &other) const;

  // Compute the difference of heights between two points
  float dist_z(const Point &other) const;

  // Compute the Euclidean distance between two points
  float dist(const Point &other) const;

  // Compute the angle between two points
  float angle_xy(const Point &other) const;

  // Check if two points are equals
  bool operator==(const Point &other);

  // Convert a point in string form
  friend std::ostream &operator<<(std::ostream &os, const Point &pnt);
};

} // namespace nav

#endif /* POINT_H */
