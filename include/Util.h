/**
 * @file Util.h
 * @brief Header file for class Util
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef UTIL_H
#define UTIL_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <iostream>
#include <list>
#include <vector>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

typedef std::pair<size_t, float> WtEdge;

#define INF 1000000000.0f

// Convert three-dimensional indexes into a linear index according to the given

size_t sub_to_ind_xy(std::vector<size_t> &size, std::vector<size_t> &idxs);
size_t sub_to_ind(std::vector<size_t> &size, std::vector<size_t> &idxs);

// Convert a linear index into three-dimensional indexes according to the given
// size
std::vector<size_t> *ind_to_sub_xy(std::vector<size_t> &size, size_t ind);
std::vector<size_t> *ind_to_sub(std::vector<size_t> &size, size_t ind);

// Given an index and a level find its neighbors
std::vector<size_t> *find_neighs(std::vector<size_t> &size, size_t ind,
                                 int xy_level, int z_level);

// Given a certain index find its links
std::vector<size_t> *find_links_xy(std::vector<size_t> &size, size_t ind);
std::vector<size_t> *find_links(std::vector<size_t> &size, size_t ind);

// Round a float
static inline __attribute__((always_inline)) float round(float val) {
  float value = (int)(val * 100000);
  return (float)value / 100000;
};

} // namespace nav

#endif /* UTIL_H */