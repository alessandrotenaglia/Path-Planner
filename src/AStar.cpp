/**
 * @file AStar.cpp
 * @brief Source file for class AStar
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "AStar.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize A*
AStar::AStar(Map map) : map_(map), vxs_(map.n()), str_(NULL), trg_(NULL) {
  // Loop on vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    // Set the box index
    this->vxs_[ind].set_ind(ind);
  }
}

// Set start vertex from point
void AStar::set_str(const Point &str_pnt) {
  this->set_str(this->map_.pnt_to_ind(str_pnt));
}

// Set start vertex from index
void AStar::set_str(size_t str_ind) {
  // Check start index
  if (str_ind >= this->map_.n())
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->map_.boxes()[str_ind].free() || !this->map_.boxes()[str_ind].in())
    throw "ERROR: Start box is not free!";
  // Set start vertex
  this->str_ = &this->vxs_[str_ind];
}

// Set target vertex
void AStar::set_trg(const Point &trg_pnt) {
  // Check target point
  size_t trg_ind = this->map_.pnt_to_ind(trg_pnt);
  if (trg_ind >= this->map_.n())
    throw "ERROR: Target point is out of map!";
  // Check target box
  if (!this->map_.boxes()[trg_ind].free() || !this->map_.boxes()[trg_ind].in())
    throw "ERROR: Target box is not free!";
  // Set target vertex
  this->trg_ = &this->vxs_[trg_ind];
}

// Get path
std::list<const Box *> *AStar::path(size_t curr_ind) {
  auto *path = new std::list<const Box *>;
  ASVx *vx = this->trg_;
  while (vx->ind() != curr_ind) {
    path->push_front(&this->map_.boxes(vx->ind()));
    vx = vx->get_pred();
    if (vx == NULL)
      throw "ERROR: No path found";
  }
  return path;
}

// !! There is a bug
void AStar::compute_shortest_path() {
  // Initialize vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    ASVx *vx = &this->vxs_[ind];
    vx->set_f(INF);
    vx->set_g(INF);
    if (this->map_.boxes(vx->ind()).free() &&
        this->map_.boxes(vx->ind()).in()) {
      vx->set_h(this->map_.boxes(vx->ind()).cnt().dist(
          this->map_.boxes(this->trg_->ind()).cnt()));
    } else {
      vx->set_h(INF);
    }
    vx->set_pred(NULL);
  }
  // Initialize OPEN and close set
  std::set<ASVx *, ASVxCmp> OPEN;
  std::unordered_set<ASVx *> CLOSED;
  // Setup start vertex
  this->str_->set_f(this->str_->h());
  this->str_->set_g(0.0f);
  OPEN.insert(this->str_);
  // Loop on OPEN set
  while (!OPEN.empty()) {
    // Pop first vertex from the OPEN set and add it to the CLOSED set
    ASVx *curr = *OPEN.begin();
    OPEN.erase(curr);
    CLOSED.insert(curr);
    // Check if the target has been reached
    if (curr->ind() == this->trg_->ind()) {
      return;
    }
    // Loop on edges
    for (WtEdge edge : this->map_.boxes(curr->ind()).edges()) {
      // Link
      ASVx *link = &this->vxs_[edge.first];
      // Cost to reach the link passing through the current vertex
      float g_score = curr->g() + edge.second;
      // Check if the vertex is in the OPEN set
      bool in_OPEN = (OPEN.find(link) != OPEN.end());
      // Check if the vertex is in the CLOSED set
      bool in_CLOSED = (CLOSED.find(link) != CLOSED.end());
      // If it has been reached for the first time, it's added to OPEN set
      if (!in_OPEN && !in_CLOSED) {
        link->set_f(g_score + link->h());
        link->set_g(g_score);
        link->set_pred(curr);
        OPEN.insert(link);
      }
      // Compare the new path with the one already found
      else {
        // New path is better
        if (g_score < link->g()) {
          if (in_OPEN)
            OPEN.erase(link);
          if (in_CLOSED)
            CLOSED.erase(link);
          link->set_f(g_score);
          link->set_g(g_score);
          link->set_pred(curr);
          OPEN.insert(link);
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

void AStar::update(size_t curr_ind, std::list<Point> slam_pntcloud) {
  if (this->map_.slam_update(slam_pntcloud)) {
    // TODO update only updated box
    this->set_str(curr_ind);
    this->compute_shortest_path();
  }
}

} // namespace nav