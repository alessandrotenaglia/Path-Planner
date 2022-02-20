/**
 * @file NavMap.h
 * @brief Header file for class NavMap
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef NAVMAP_H
#define NAVMAP_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Point.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class NavBox {
private:
  size_t ind_;                 // Linear index
  Point cnt_;                  // Center
  bool in_;                    // Flag inside/outside map
  bool free_;                  // Flag free/busy
  std::list<Point> fix_pnts_;  // Fixed points
  std::list<Point> slam_pnts_; // SLAM points
  std::vector<size_t> neighs_; // Neighbors
  std::vector<WtEdge> edges_;  // Edges

  // Box serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &ind_ &cnt_ &in_ &free_ &fix_pnts_ &slam_pnts_ &neighs_ &edges_;
  }

public:
  // Default constructor
  NavBox() : ind_(-1), cnt_(), in_(false), free_(true){};

  // Set ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set center
  void set_cnt(Point &cnt) { cnt_ = cnt; }
  // Get box center
  const Point &cnt() const { return cnt_; }

  // Set inside
  void set_in(bool in) { in_ = in; }
  // Get inside
  const bool &is_in() const { return in_; }

  // Set free
  void set_free() { free_ = true; }
  void set_busy() { free_ = false; }
  // Get free
  const bool &is_free() const { return free_; }

  // Add a fixed point inside the box
  void add_fix_pnt(const Point &pnt) { fix_pnts_.push_back(pnt); }
  // Get fixed points inside the box
  const std::list<Point> &fix_pnts() const { return fix_pnts_; }

  // Add a SLAM point inside the box
  void add_slam_pnt(const Point &pnt) { slam_pnts_.push_back(pnt); }
  // Get SLAM points inside the box
  const std::list<Point> &slam_pnts() const { return slam_pnts_; }
  // Remove SLAM points
  void remove_slam_pnts() { slam_pnts_.clear(); }

  // Set neighbors
  void set_neighs(std::vector<size_t> &neighs) { neighs_ = neighs; }
  // Get neighbors
  const std::vector<size_t> &neighs() const { return neighs_; }

  // Add an edge
  void add_edge(size_t dest, float wt) {
    edges_.push_back(std::make_pair(dest, wt));
  }
  // Get edges
  const std::vector<WtEdge> &edges() const { return edges_; }
};

class NavMap {
private:
  float xlen_, ylen_, zlen_;    // Map dimension
  size_t nx_, ny_, nz_, n_;     // Number of boxes
  float xstep_, ystep_, zstep_; // Steps length
  float radius_, height_;       // Drone dimensions
  std::vector<NavBox> boxes_;      // Vector containing all boxes
  std::vector<bool> updatable_; // Indexes that can be updated

  // Nav Map serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &xlen_ &ylen_ &zlen_ &nx_ &ny_ &nz_ &n_ &xstep_ &ystep_ &zstep_ &radius_
        &height_ &boxes_ &updatable_;
  }

public:
  // Default constructor
  NavMap() {}

  // Initialize a map
  NavMap(float xlen, float ylen, float zlen, size_t nx, size_t ny, size_t nz,
      float radius, float height, std::list<Point> fix_pntcloud);

  // Get number of Boxes
  const size_t &n() const { return n_; }

  // Get boxes
  const std::vector<NavBox> &boxes() const { return boxes_; }

  // Get ind-th box
  const NavBox &boxes(size_t ind) const { return boxes_[ind]; }

  // Get updatable
  bool is_updatable(size_t ind) { return updatable_[ind]; }

  // Update map from SLAM pointcloud
  std::list<size_t> slam_update(std::list<Point> slam_pntcloud);

  // Compute the index of the corresponding box
  size_t pnt_to_ind(const Point &pnt);
};

} // namespace nav

#endif /* NAVMAP_H */
