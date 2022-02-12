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
  Map map_;               //
  std::vector<ASVx> vxs_; //
  ASVx *str_;             //
  ASVx *trg_;             //
  ASVx *curr_;            //

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
  ASVx *str() { return str_; };

  // Set target vertex
  void set_trg(const Point &trg_pnt);
  // Get target vertex
  ASVx *trg() { return trg_; };

  // Get path
  std::list<Box *> *path(size_t curr_ind);

  void compute_shortest_path();

  void update(size_t curr_ind, std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* ASTAR_H */