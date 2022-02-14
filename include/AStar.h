/**
 * @file AStar.h
 * @brief Header file for class AStar
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef ASTAR_H
#define ASTAR_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "ASVx.h"
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class AStar {
private:
  Map map_;               // Map
  std::vector<ASVx> vxs_; // Vertices
  size_t str_;            // Start vertex
  size_t trg_;            // Target vertex

public:
  // Default constructor
  AStar(){};

  // Initialize A*
  AStar(Map map);

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
  std::list<const Box *> *path(size_t curr_ind);

  void compute_shortest_path();

  void update(size_t curr_ind, std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* ASTAR_H */