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

// Convert a box in string form
std::ostream &operator<<(std::ostream &os, const Box &box) {
  os << "[" << box.ind() << "]: " << box.cnt();
  return os;
}

} // namespace nav