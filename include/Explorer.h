/**
 * @file Explorer.h
 * @brief Header file for class Explorer
 * @date 20 February 2022
 * @author Alessandro Tenaglia
 */

#ifndef EXPLORER_H_
#define EXPLORER_H_

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Box.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class Explorer {
private:
  float xlen_, ylen_;         // Map dimension
  size_t nx_, ny_, n_;        // Number of boxes
  float xstep_, ystep_;       // Steps length
  float radius_;              // Drone dimensions
  std::vector<ExpBox> boxes_; // Vector containing all boxes

  // Exp Map serialization
  friend class boost::serialization::access;
  template <typename Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar &xlen_ &ylen_ &nx_ &ny_ &n_ &xstep_ &ystep_ &radius_ &boxes_;
  }

public:
  // Default constructor
  Explorer(){};

  // Initialize Explorator
  Explorer(float xlen, float ylen, size_t nx, size_t ny, float radius,
           std::list<Point> exp_fix_pntcloud);

  // Get ind-th box
  const ExpBox &boxes(size_t ind) const { return boxes_[ind]; }
  // Get boxes
  const std::vector<ExpBox> &boxes() const { return boxes_; }

  //
  size_t choose_dir(size_t curr_ind, size_t curr_dir);
  //
  std::vector<size_t> generate_path(size_t curr_ind, size_t curr_dir);

  // Compute the index of the corresponding box
  std::vector<size_t> *pnt_to_sub(const Point &pnt);
  size_t pnt_to_ind(const Point &pnt);
};

} // namespace nav

#endif /* EXPLORER_H_ */
