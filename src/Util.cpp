/**
 * @file Util.cpp
 * @brief Source file for class Util
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Util.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace util {

// Convert three-dimensional indexes into a linear index according to the given
// size
size_t sub_to_ind(std::vector<size_t> &size, std::vector<size_t> &idxs) {
  // Number of linear indexes
  size_t n = size[0] * size[1] * size[2];
  // Check if idxs are valid wrt size
  if ((idxs[0] < 0 || idxs[0] >= size[0]) ||
      (idxs[1] < 0 || idxs[1] >= size[1]) ||
      (idxs[2] < 0 || idxs[2] >= size[2]))
    return n;
  // Convert three-dimensional indexes to corresponding liner index
  return idxs[2] + size[2] * (idxs[1] + size[1] * idxs[0]);
}

// Convert a linear index into three-dimensional indexes according to the given
// size
std::vector<size_t> *ind_to_sub(std::vector<size_t> &size, size_t ind) {
  // Number of linear indexes
  size_t n = size[0] * size[1] * size[2];
  // Check if ind is valid wrt size
  if (ind < 0 || ind >= n)
    return NULL;
  // Convert linear index to corresponding three-dimensional indexes
  std::vector<size_t> *idxs = new std::vector<size_t>(3);
  idxs->at(2) = ind % size[2];
  size_t temp = (ind - idxs->at(2)) / size[2];
  idxs->at(1) = temp % size[1];
  idxs->at(0) = (temp - idxs->at(1)) / size[1];
  return idxs;
}

// Given an index and a level find its neighbors
std::vector<size_t> *find_neighs(std::vector<size_t> &size, size_t ind,
                                 int xy_level, int z_level) {
  // Number of linear indexes
  size_t n = size[0] * size[1] * size[2];
  // Convert linear index to corresponding tree-dimensional indexes
  std::vector<size_t> *idxs = util::ind_to_sub(size, ind);
  if (idxs == NULL)
    return NULL;
  // Find neighbors
  size_t neigh_ind;
  std::vector<size_t> neigh_idxs(3);
  std::vector<size_t> *neighbors = new std::vector<size_t>;
  for (int i = -xy_level; i <= xy_level; i++) {
    for (int j = -xy_level; j <= xy_level; j++) {
      for (int k = -z_level; k <= z_level; k++) {
        // Compute neighbor's three-dimensional indexes
        neigh_idxs[0] = idxs->at(0) + i;
        neigh_idxs[1] = idxs->at(1) + j;
        neigh_idxs[2] = idxs->at(2) + k;
        // Convert to corresponding linear index
        neigh_ind = util::sub_to_ind(size, neigh_idxs);
        // If neighbor's index is valid append to the array
        if (neigh_ind < n)
          neighbors->push_back(neigh_ind);
      }
    }
  }
  return neighbors;
}

// Given a certain index find its links
std::vector<size_t> *find_links(std::vector<size_t> &size, size_t ind) {
  // Number of linear indexes
  size_t n = size[0] * size[1] * size[2];
  // Convert linear index to corresponding tree-dimensional indexes
  std::vector<size_t> *idxs = util::ind_to_sub(size, ind);
  if (idxs == NULL)
    return NULL;
  // Find links
  size_t link_ind;
  std::vector<size_t> link_idxs(3);
  std::vector<size_t> *links = new std::vector<size_t>;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -1; k <= 1; k++) {
        // Skip same index
        if (i == 0 && j == 0 && k == 0)
          continue;
        // Skip corners
        if (i != 0 && j != 0 && k != 0)
          continue;
        // Skip edges
        if ((i != 0 || j != 0) && k != 0)
          continue;
        // Compute neighbor's three-dimensional indexes
        link_idxs[0] = idxs->at(0) + i;
        link_idxs[1] = idxs->at(1) + j;
        link_idxs[2] = idxs->at(2) + k;
        // Convert to corresponding linear index
        link_ind = util::sub_to_ind(size, link_idxs);
        // If neighbor's index is valid append to the array
        if (link_ind < n)
          links->push_back(link_ind);
      }
    }
  }
  return links;
}

} // namespace util