/**
 * @file Box.h
 * @brief Header file for class Box
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef BOX_H_
#define BOX_H_

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
protected:
  size_t ind_;                 // Linear index
  Point cnt_;                  // Center
  bool in_;                    // Flag inside/outside map
  bool free_;                  // Flag free/busy
  std::vector<WtEdge> edges_;  // Edges
  std::vector<size_t> neighs_; // Neighbors
  std::list<Point> fix_pnts_;  // Fixed points

  // Box serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &ind_ &cnt_ &in_ &free_ &edges_ &neighs_ &fix_pnts_;
  }

public:
  // Default constructor
  ExpBox() : ind_(-1), cnt_(), in_(false), free_(true) {}

  // Set ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set center
  void set_cnt(Point &cnt) { cnt_ = cnt; }
  // Get box center
  const Point &cnt() const { return cnt_; }

  // Set inside
  void set_in() { in_ = true; }
  void set_out() { in_ = false; }
  // Get inside
  const bool &is_in() const { return in_; }

  // Set free
  void set_free() { free_ = true; }
  void set_busy() { free_ = false; }
  // Get free
  const bool &is_free() const { return free_; }

  // Add an edge
  void add_edge(size_t dest, float wt, size_t dir) {
    edges_.push_back(std::make_tuple(dest, wt, dir));
  }
  // Get edges
  const std::vector<WtEdge> &edges() const { return edges_; }

  // Set neighbors
  void set_neighs(std::vector<size_t> &neighs) { neighs_ = neighs; }
  // Get neighbors
  const std::vector<size_t> &neighs() const { return neighs_; }

  // Add a fixed point inside the box
  void add_fix_pnt(const Point &pnt) { fix_pnts_.push_back(pnt); }
  // Get fixed points inside the box
  const std::list<Point> &fix_pnts() const { return fix_pnts_; }
};

class NavBox : public ExpBox {
private:
  std::list<Point> slam_pnts_; // SLAM points
  float g_;
  float h_;
  size_t pred_;

  // Box serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &ind_ &cnt_ &in_ &free_ &edges_ &neighs_ &fix_pnts_ &slam_pnts_ &g_ &h_
        &pred_;
  }

public:
  // Default constructor
  NavBox() : ExpBox(), g_(INF), h_(INF), pred_(-1) {}

  // Add a SLAM point inside the box
  void add_slam_pnt(const Point &pnt) { slam_pnts_.push_back(pnt); }
  // Remove SLAM points
  void rm_slam_pnts() { slam_pnts_.clear(); }
  // Get SLAM points inside the box
  const std::list<Point> &slam_pnts() const { return slam_pnts_; }

  // Set g value
  void set_g(float g) { g_ = g; }
  // Get g value
  const float &g() const { return g_; }

  // Set h value
  void set_h(float h) { h_ = h; }
  // Get h value
  const float &h() const { return h_; }

  // Get f value
  float get_f() { return (g_ + h_); };

  // Set predecessor
  void set_pred(size_t pred) { pred_ = pred; }
  // Get predecessor
  size_t pred() { return pred_; }
};

class Node {
private:
  size_t ind_;
  float f_;

public:
  // Default constructor
  Node() {}

  // Initialize node
  Node(size_t ind, float f) : ind_(ind), f_(f) {}

  // Get box ind
  const size_t &ind() const { return ind_; }

  // Get f value
  const float &f() const { return f_; }

  // Operator >
  bool operator>(const Node &rhs) const { return (f_ > rhs.f()); }

  // Operator <
  bool operator<(const Node &rhs) const { return (f_ < rhs.f()); }

  // Operator ==
  bool operator==(const Node &rhs) const { return (ind_ == rhs.ind()); }

  // Convert a node in string form
  friend std::ostream &operator<<(std::ostream &os, const Node &vx) {
    os << "[ind: " << vx.ind() << "; f: " << vx.f() << "]";
    return os;
  }
};

} // namespace nav

#endif /* BOX_H_ */
