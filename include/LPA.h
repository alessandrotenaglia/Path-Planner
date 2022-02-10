/**
 * @file LPA.h
 * @brief Header file for class LPA
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef LPA_H
#define LPA_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "LPAVx.h"
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

class LPA {
private:
  Map map_;
  std::vector<LPAVx> vxs_;
  LPAVx *str_;
  LPAVx *trg_;
  std::set<LPAVx *, LPAVxCmp> open_;

public:
  // Default constructor
  LPA(){};

  // Initialize LPA
  LPA(Map map);

  // Get map
  const Map &map() const { return map_; };

  // Set start vertex
  void set_str(const Point &p_str);
  // Get start vertex
  LPAVx *str() { return str_; };

  // Set target vertex
  void set_trg(const Point &p_trg);
  // Get target vertex
  LPAVx *trg() { return trg_; };

  void initialize();

  void update_vx(LPAVx *vx);

  void compute_shortest_path();

  std::list<Box *> *path(size_t src_ind);

  void update_map(std::list<Point> slam_pntcloud);
};

} // namespace nav

#endif /* LPA_H */