/**
 * @file Map.h
 * @brief Header file for class Map
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef MAP_H
#define MAP_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <limits>
#include <queue>
#include <set>
#include <unordered_set>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Box.h"
#include "Util.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class Map {
private:
  float xlen_, ylen_, zlen_;    // Map dimension
  size_t nx_, ny_, nz_, n_;     // Number of boxes
  float xstep_, ystep_, zstep_; // Steps length
  float radius_, height_;       // Drone dimensions
  std::vector<Box> boxes_;      // Vector containing all boxes
  std::vector<bool> updatable_; // Vector containing box indices updatable
  std::vector<bool> to_update_; // Vector containing box indices to update
  std::vector<bool> updated_;   // Vector containing box indices updated

  // Map serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &xlen_ &ylen_ &zlen_ &nx_ &ny_ &nz_ &n_ &xstep_ &ystep_ &zstep_ &radius_
        &height_ &boxes_ &updatable_ &to_update_ &updated_;
  }

public:
  // Default constructor
  Map(){};

  // Initialize a map
  Map(float xlen, float ylen, float zlen, size_t nx, size_t ny, size_t nz,
      float radius, float height, std::list<Point> pntcloud);

  // Set number of boxes
  void set_n(size_t n) { n_ = n; }
  // Get number of Boxes
  const size_t &n() const { return n_; }

  // Set boxes
  std::vector<Box> &boxes() { return boxes_; }
  // Get boxes
  const std::vector<Box> &boxes() const { return boxes_; }

  // Set ind-th box
  Box &boxes(size_t ind) { return boxes_[ind]; }
  // Get ind-th box
  const Box &boxes(size_t ind) const { return boxes_[ind]; }

  // Assign SLAM points to the respective boxes
  void add_slam_pntcloud(std::list<Point> slam_pntcloud);

  // Set SLAM obsatcles
  void set_slam_obstacles();

  // Update map from SLAM pointcloud
  void slam_update(std::list<Point> slam_pntcloud);

  // Compute the index of the corresponding box
  size_t pnt_to_ind(const Point &pnt);
};

} // namespace nav

#endif /* MAP_H */