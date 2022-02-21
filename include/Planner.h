/**
 * @file Planner.h
 * @brief Header file for class Planner
 * @date 20 February 2022
 * @author Alessandro Tenaglia
 */

#ifndef PLANNER_H
#define PLANNER_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <unordered_set>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Box.h"
#include "FibonacciHeap.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class Planner {
private:
  float xlen_, ylen_, zlen_;    // Map dimension
  size_t nx_, ny_, nz_, n_;     // Number of boxes
  float xstep_, ystep_, zstep_; // Steps length
  float radius_, height_;       // Drone dimensions
  std::vector<Box> boxes_;      // Vector containing all boxes
  std::vector<bool> updatable_; // Indexes that can be updated
  size_t str_;                  // Start box
  size_t trg_;                  // Target box
  std::list<size_t> path_;      // Shortest path

  // Nav Map serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &xlen_ &ylen_ &zlen_ &nx_ &ny_ &nz_ &n_ &xstep_ &ystep_ &zstep_ &radius_
        &height_ &boxes_ &updatable_ &str_ &trg_ &path_;
  }

public:
  // Default constructor
  Planner() {}

  // Initialize a map
  Planner(float xlen, float ylen, float zlen, size_t nx, size_t ny, size_t nz,
          float radius, float height, std::list<Point> fix_pntcloud);

  // Get ind-th box
  const Box &boxes(size_t ind) const { return boxes_[ind]; }
  // Get boxes
  const std::vector<Box> &boxes() const { return boxes_; }

  // Set start box from point
  void set_str(const Point &str_pnt);
  // Get start box
  const size_t &str() const { return str_; };

  // Set target box
  void set_trg(const Point &trg_pnt);
  // Get target box
  const size_t &trg() const { return trg_; };

  // Compute shortest path
  void search();
  // Set path
  void set_path();
  // Get path
  const std::list<size_t> &path() const { return this->path_; };

  // Update map with SLAM pointcloud
  void update(std::list<Point> slam_pntcloud);

  //
  size_t move();

  // Compute the index of the corresponding box
  size_t pnt_to_ind(const Point &pnt);
};

} // namespace nav

#endif /* PLANNER_H */