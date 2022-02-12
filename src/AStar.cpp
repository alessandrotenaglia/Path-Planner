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
  // Loop on map boxes
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    // Set the box index
    this->vxs_[ind].set_ind(ind);
  }
}

// Set start vertex from point
void AStar::set_str(const Point &str_pnt) {
  // Check start point
  size_t str_ind = this->map_.pnt_to_ind(str_pnt);
  if (str_ind >= this->map_.n())
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->map_.boxes()[str_ind].free())
    throw "ERROR: Start box is not free!";
  // Set start vertex
  this->str_ = &this->vxs_[str_ind];
}

// Set start vertex from index
void AStar::set_str(size_t str_ind) {
  // Check start point
  if (str_ind >= this->map_.n())
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->map_.boxes()[str_ind].free())
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
  if (!this->map_.boxes()[trg_ind].free())
    throw "ERROR: Target box is not free!";
  // Set target vertex
  this->trg_ = &this->vxs_[trg_ind];
  // Initialize vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    float h = INF;
    if (this->map_.boxes(ind).free())
      h = this->map_.boxes(ind).cnt().dist(
          this->map_.boxes(this->trg_->ind()).cnt());
    this->vxs_[ind].init(h);
  }
}

// Get path
std::list<Box *> *AStar::path(size_t curr_ind) {
  auto *path = new std::list<Box *>;
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
  // Initialize open and close set
  auto *open = new std::set<ASVx *, ASVxCmp>();
  auto *closed = new std::unordered_set<size_t>();
  // Setup start vertex
  this->str_->set_g(0.0f);
  this->str_->set_f(this->str_->h());
  open->insert(this->str_);
  // Start search
  ASVx *curr, *neigh;
  float g_score;
  bool in_open, in_closed;
  // Loop on open set
  while (!open->empty()) {
    // Pop fist box in the open set and add it in the closed set
    curr = *open->begin();
    open->erase(curr);
    closed->insert(curr->ind());
    // Check if the target has been reached
    if (curr->ind() == this->trg_->ind()) {
      return;
    }
    // Loop on edges
    for (WtEdge edge : this->map_.boxes(curr->ind()).edges()) {
      // Neighbor
      neigh = &this->vxs_[edge.first];
      // Check if the target has been reached
      if (neigh->ind() == this->trg_->ind()) {
        // Set path
        this->trg_->set_pred(curr);
        return;
      }
      // Cost to reach the neighbor passing through the current vertex
      g_score = curr->g() + edge.second;
      // Check if the vertex is in the open set
      in_open = open->find(neigh) != open->end();
      // Check if the vertex is in the closed set
      in_closed = closed->find(neigh->ind()) != closed->end();
      // If it has been reached for the first time, it's added to open set
      if (!in_open && !in_closed) {
        neigh->set_pred(curr);
        neigh->set_g(g_score);
        neigh->set_f(neigh->g() + neigh->h());
        open->insert(neigh);
      }
      // Compare the new path with the one already found
      else {
        // New path is better
        if (g_score < neigh->g()) {
          if (in_open)
            open->erase(neigh);
          if (in_closed)
            closed->erase(neigh->ind());
          neigh->set_pred(curr);
          neigh->set_g(g_score);
          neigh->set_f(neigh->g() + neigh->h());
          open->insert(neigh);
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

void AStar::update(size_t curr_ind, std::list<Point> slam_pntcloud) {
  if (this->map_.slam_update(slam_pntcloud) > 0) {
    // TODO update only updated box
    // Initialize vertices
    for (size_t ind = 0; ind < this->map_.n(); ind++) {
      if (this->map_.updated(ind)) {
        float h = INF;
        if (this->map_.boxes(ind).free())
          h = this->map_.boxes(ind).cnt().dist(
              this->map_.boxes(this->trg_->ind()).cnt());
        this->vxs_[ind].init(h);
      }
    }
    this->set_str(curr_ind);
    this->compute_shortest_path();
  }
}

} // namespace nav