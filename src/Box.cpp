/**
 * @file Box.cpp
 * @brief Source file for class Box
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Box.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Add a fixed point inside the box
void Box::add_fix_pnt(const Point &pnt) { this->fix_pnts_.push_back(pnt); }

// Add a SLAM point inside the box
void Box::add_slam_pnt(const Point &pnt) { this->slam_pnts_.push_back(pnt); }

// Add an edge
void Box::add_edge(size_t dest, float wt) {
  this->edges_.push_back(std::make_pair(dest, wt));
}

// Convert a box in string form
std::ostream &operator<<(std::ostream &os, const Box &box) {
  os << "[" << box.ind() << "]: " << box.cnt();
  return os;
}

} // namespace nav