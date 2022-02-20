/**
 * @file ExpMap.h
 * @brief Header file for class ExpMap
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef EXPMAP_H
#define EXPMAP_H

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

class ExpBox {
private:
  size_t ind_;
  Point cnt_;
  bool free_;                 // Flag free/busy
  std::list<Point> fix_pnts_; // Fixed points
  std::vector<WtEdge> edges_;
  bool explored_;
  size_t f_;

  // Box serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &ind_ &cnt_ &free_ &fix_pnts_ &edges_ &explored_ &f_;
  }

public:
  // Default constructor
  ExpBox() : ind_(-1), cnt_(), free_(true), explored_(false), f_(-1) {}

  // Set ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set center
  void set_cnt(Point &cnt) { cnt_ = cnt; }
  // Get box center
  const Point &cnt() const { return cnt_; }

  // Set free
  void set_free() { free_ = true; }
  void set_busy() { free_ = false; }
  // Get free
  const bool &is_free() const { return free_; }

  // Add a fixed point inside the box
  void add_fix_pnt(const Point &pnt) { fix_pnts_.push_back(pnt); }
  // Get fixed points inside the box
  const std::list<Point> &fix_pnts() const { return fix_pnts_; }

  // Set status
  void set_explored() { explored_ = true; }
  // Get free
  const bool &is_explored() const { return explored_; }

  // Set free
  void set_f(size_t f) { f_ = f; }
  // Get free
  const size_t &f() const { return f_; }

  // Add an edge
  void add_edge(size_t dest, float wt) {
    edges_.push_back(std::make_pair(dest, wt));
  }
  // Get edges
  const std::vector<WtEdge> &edges() const { return edges_; }
};

class ExpMap {
private:
  float xlen_, ylen_;         // Map dimension
  size_t nx_, ny_, n_;        // Number of boxes
  float xstep_, ystep_;       // Steps length
  float radius_;              // Drone dimensions
  std::vector<ExpBox> boxes_; //

  // Exp Map serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &xlen_ &ylen_ &nx_ &ny_ &n_ &xstep_ &ystep_ &radius_ &boxes_;
  }

public:
  // Default constructor
  ExpMap(){};

  // Initialize Explorator
  ExpMap(float xlen, float ylen, size_t nx, size_t ny, float radius,
         std::list<Point> exp_fix_pntcloud);

  // Get ind-th box
  const ExpBox &boxes(size_t ind) const { return boxes_[ind]; }

  // Get boxes
  const std::vector<ExpBox> &boxes() const { return boxes_; }

  //
  void set_explored(size_t ind) {
    boxes_[ind].set_explored();
    for (nav::WtEdge edge : this->boxes_[ind].edges()) {
      this->boxes_[edge.first].set_f(this->boxes_[edge.first].f() - 1);
    }
  }

  // Compute the index of the corresponding box
  size_t pnt_to_ind(const Point &pnt);
};

} // namespace nav

#endif /* EXPMAP_H */
