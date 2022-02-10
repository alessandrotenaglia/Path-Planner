/**
 * @file LPAVx.h
 * @brief Header file for class LPAVx
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef LPAVX_H
#define LPAVX_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <iostream>
#include <limits>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

#define INF std::numeric_limits<float>::max()

class LPAVx {
private:
  size_t ind_; //
  float g_;    //
  float h_;    //
  float rhs_;  //
  float k1_;   //
  float k2_;   //

public:
  // Default constructor
  LPAVx() : ind_(-1), g_(INF), h_(INF), rhs_(INF), k1_(INF), k2_(INF){};

  // Set box ind
  void set_ind(size_t ind) { ind_ = ind; }
  // Get box ind
  const size_t &ind() const { return ind_; }

  // Set g value
  void set_g(float g) { g_ = g; }
  // Get g value
  const float &g() const { return g_; }

  // Set h value
  void set_h(float h) { h_ = h; }
  // Get h value
  const float &h() const { return h_; }

  // Set rhs value
  void set_rhs(float rhs) { rhs_ = rhs; }
  // Get rhs value
  const float &rhs() const { return rhs_; }

  // Set key
  void calculate_key() {
    k1_ = std::min(g_, rhs_ + h_);
    k2_ = std::min(g_, rhs_);
  }
  // Get key
  const float &k1() const { return k1_; }
  const float &k2() const { return k2_; }

  // Init vertex values
  void init(float h) {
    g_ = INF;
    h_ = h;
    rhs_ = INF;
    k1_ = INF;
    k2_ = INF;
  };

  // < operator
  bool operator<(const LPAVx &other) {
    if (this->k1() < other.k1())
      return true;
    else if (this->k1() == other.k1())
      return this->k2() < other.k2();
    else
      return false;
  }
};

// Custom comparator
struct LPAVxCmp {
  bool operator()(const LPAVx *lhs, const LPAVx *rhs) const {
    if (lhs->k1() < rhs->k1())
      return true;
    else if (lhs->k1() == rhs->k1())
      return lhs->k2() < rhs->k2();
    else
      return false;
  }
};

} // namespace nav

#endif /* LPAVX_H */