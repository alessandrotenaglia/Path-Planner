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
  Map map_;                           //
  std::vector<ASVx> vxs_;             //
  ASVx *curr_;                        //
  ASVx *str_;                         //
  ASVx *trg_;                         //
  std::set<ASVx *, ASVxCmp> open_;    //
  std::unordered_set<size_t> closed_; //

public:
  // Default constructor
  AStar(){};

  // Initialize A*
  AStar(Map map);

  // Get map
  const Map &map() const { return map_; };

  // Set curr vertex
  void set_curr(size_t curr_ind);
  // Get curr vertex
  ASVx *curr() { return curr_; };

  // Set start vertex
  void set_str(const Point &p_str);
  // Get start vertex
  ASVx *str() { return str_; };

  // Set target vertex
  void set_trg(const Point &p_trg);
  // Get target vertex
  ASVx *trg() { return trg_; };

  void initialize();

  void compute_shortest_path();

  std::list<Box *> *path(size_t src_ind);

  void update(std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* ASTAR_H */