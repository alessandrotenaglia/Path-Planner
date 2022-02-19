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
  float k1_;   // Priority value
  float k2_;   // Priority value

public:
  // Default constructor
  LPANode() : ind_(-1), k1_(INF), k2_(INF){};

  // Default constructor
  LPANode(size_t ind, float k1, float k2) : ind_(ind), k1_(k1), k2_(k2){};

  // Set box ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set key
  void set_k1(float k1) { k1_ = k1; }
  void set_k2(float k2) { k2_ = k2; }
  // Get key
  const float &k1() const { return k1_; }
  const float &k2() const { return k2_; }

  // Operator <
  bool operator<(LPANode rhs) const {
    if (k1_ < rhs.k1()) {
      return true;
    } else if (k1_ == rhs.k1()) {
      return k2_ < rhs.k2();
    } else {
      return false;
    }
  }

  // Operator >
  bool operator>(LPANode rhs) const {
    if (k1_ > rhs.k1()) {
      return true;
    } else if (k1_ == rhs.k1()) {
      return k2_ > rhs.k2();
    } else {
      return false;
    }
  }

  // Operator ==
  bool operator==(LPANode rhs) const {
    return (k1_ == rhs.k1()) & (k2_ == rhs.k2());
  }

  // Convert a ASNode in string form
  friend std::ostream &operator<<(std::ostream &os, const LPANode &vx) {
    os << "[ind: " << vx.ind() << "; k1: " << vx.k1() << "; k2: " << vx.k2()
       << "]";
    return os;
  }
};

class LPAVertex {
private:
  size_t ind_; // Linear index
  float g_;    // Cost to vertex
  float rhs_;  // Cost to vertex
  float k1_;   // Priority value
  float k2_;   // Priority value
  float h_;    // Heuristic to target

public:
  // Default constructor
  LPAVertex() : ind_(-1), g_(INF), rhs_(INF), k1_(INF), k2_(INF), h_(INF) {};

  // Set box ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set g value
  void set_g(float g) { g_ = g; }
  // Get g value
  const float &g() const { return g_; }

  // Set rhs value
  void set_rhs(float rhs) { rhs_ = rhs; }
  // Get rhs value
  const float &rhs() const { return rhs_; }

  // Set f value
  void calculate_key() {
    k1_ = std::min(g_, rhs_ + h_);
    k2_ = std::min(g_, rhs_);
  }
  // Get f value
  const float &k1() const { return k1_; }
  const float &k2() const { return k2_; }

  // Set h value
  void set_h(float h) { h_ = h; }
  // Get h value
  const float &h() const { return h_; }

  // Get node
  LPANode node(){
    calculate_key();
    return LPANode(ind_, k1_, k2_);
  }

  // Operator <
  bool operator<(LPAVertex &rhs) const {
    if (k1_ < rhs.k1()) {
      return true;
    } else if (k1_ == rhs.k1()) {
      return k2_ < rhs.k2();
    } else {
      return false;
    }
  }

  // Convert a ASNode in string form
  friend std::ostream &operator<<(std::ostream &os, const LPAVertex &vx) {
    os << "[ind: " << vx.ind() << "; k1: " << vx.k1() << "; k2: " << vx.k2()
       << "]";
    return os;
  }
};

class LPAStar {
private:
  Map map_;                    // Map
  std::vector<LPAVertex> vxs_; // Vertices
  size_t str_;                 // Start vertex
  size_t trg_;                 // Target vertex
  FibonacciHeap<LPANode> OPEN; // Priority queue

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

  // Get path
  std::list<size_t> path(size_t curr_ind);

  // Initialize
  void initialize();

  // Compute shortest path
  void compute_shortest_path();

  //
  void update_node(size_t ind);

  // Update map with SLAM pointcloud
  void update_map(std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* LPASTAR_H */
