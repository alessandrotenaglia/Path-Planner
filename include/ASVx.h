/**
 * @file ASVx.h
 * @brief Header file for class ASVx
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef ASVX_H
#define ASVX_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <iostream>
#include <limits>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Util.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

#define INF std::numeric_limits<float>::max()

class ASVx {
private:
  size_t ind_;  // Linear index
  float f_;     // Priority value
  float g_;     // Cost to vertex
  float h_;     // Heuristic to target
  size_t pred_; // Predecessor

public:
  // Default constructor
  ASVx() : ind_(-1), f_(INF), g_(INF), h_(INF), pred_(-1){};

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
};

// Custom comparator
struct ASVxCmp {
  bool operator()(ASVx *lhs, ASVx *rhs) const {
    return (lhs->f() < rhs->f());
  }
};

} // namespace nav

#endif /* ASVX_H */