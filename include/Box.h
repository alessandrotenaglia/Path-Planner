/**
 * @file Box.h
 * @brief Header file for class Box
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef BOX_H
#define BOX_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <boost/serialization/list.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <list>
/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Point.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

typedef std::pair<size_t, float> WtEdge;

class Box {
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
  Box() : ind_(-1), cnt_(), in_(false), free_(true){};

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

  // Convert a box in string form
  friend std::ostream &operator<<(std::ostream &os, const Box &box);
};

} // namespace nav

#endif /* BOX_H */