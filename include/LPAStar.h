/**
 * @file LPAStar.h
 * @brief Header file for class LPAStar
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef LPASTAR_H
#define LPASTAR_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "FibonacciHeap.h"
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class LPANode {
private:
  size_t ind_; // Linear index
  float f_;    // Priority value

public:
  // Default constructor
  LPANode() : ind_(-1), f_(INF){};

  // Default constructor
  LPANode(size_t ind, float f) : ind_(ind), f_(f){};

  // Set box ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set f value
  void set_f(float f) { f_ = f; }
  // Get f value
  const float &f() const { return f_; }

  // Operator >
  bool operator>(LPANode rhs) const { return (this->f() > rhs.f()); }

  // Operator <
  bool operator<(LPANode rhs) const { return (this->f() < rhs.f()); }

  // Operator ==
  bool operator==(LPANode rhs) const { return (this->f() == rhs.f()); }

  // Convert a ASNode in string form
  friend std::ostream &operator<<(std::ostream &os, const LPANode &vx) {
    os << "[ind: " << vx.ind() << "; f: " << vx.f() << "]";
    return os;
  }
};

class LPAVertex {
private:
  size_t ind_;  // Linear index
  float f_;     // Priority value
  float g_;     // Cost to vertex
  float h_;     // Heuristic to target
  size_t pred_; // Predecessor

public:
  // Default constructor
  LPAVertex() : ind_(-1), f_(INF), g_(INF), h_(INF), pred_(-1){};

  // Set box ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set f value
  void set_f() { f_ = g_ + h_; }
  // Get f value
  const float &f() const { return f_; }

  // Set g value
  void set_g(float g) { g_ = g; }
  // Get g value
  const float &g() const { return g_; }

  // Set h value
  void set_h(float h) { h_ = h; }
  // Get h value
  const float &h() const { return h_; }

  // Set predecessor
  void set_pred(size_t pred) { pred_ = pred; }
  // Get predecessor
  size_t pred() { return pred_; }

  /*// Operator >
  bool operator>(ASVertex *rhs) const { return (this->f() > rhs->f()); }

  // Operator <
  bool operator<(ASVertex *rhs) const { return (this->f() < rhs->f()); }

  // Operator ==
  bool operator==(ASVertex *rhs) const { return (this->f() == rhs->f()); }*/

  // Convert a point in string form
  friend std::ostream &operator<<(std::ostream &os, const LPAVertex &vx) {
    os << "[ind: " << vx.ind() << "; f: " << vx.f() << "]";
    return os;
  }
};

class LPAStar {
private:
  Map map_;                          // Map
  std::vector<LPAVertex> vxs_;       // Vertices
  size_t str_;                       // Start vertex
  size_t trg_;                       // Target vertex
  FibonacciHeap<LPANode> OPEN;       //
  std::unordered_set<size_t> CLOSED; //
  std::list<size_t> path_;           // Path

public:
  // Default constructor
  LPAStar(){};

  // Initialize A*
  LPAStar(Map map);

  // Get map
  const Map &map() const { return map_; };

  // Set start vertex from point
  void set_str(const Point &str_pnt);
  // Set start vertex from index
  void set_str(size_t str_ind);
  // Get start vertex
  size_t str() { return str_; };

  // Set target vertex
  void set_trg(const Point &trg_pnt);
  // Get target vertex
  size_t trg() { return trg_; };

  // Set path
  void set_path();
  // Get path
  std::list<size_t> path() { return this->path_; };

  // Compute shortest path
  void compute_shortest_path();

  //
  size_t next() {
    this->str_ = this->path_.front();
    this->path_.pop_front();
    return this->str_;
  };

  // Update map with SLAM pointcloud
  void update(std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* LPASTAR_H */