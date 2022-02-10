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

AStar::AStar(Map map) : map_(map), vxs_(map.n()), str_(NULL), trg_(NULL) {
  // Loop on map boxes
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    // Set the box index
    this->vxs_[ind].set_ind(ind);
  }
}

// Set start vertex
void AStar::set_str(const Point &p_str) {
  // Check start point
  size_t ind_str = this->map_.pnt_to_ind(p_str);
  if (ind_str >= this->map_.n())
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->map_.boxes()[ind_str].free())
    throw "ERROR: Start box is not free!";
  // Set start vertex
  this->str_ = &this->vxs_[ind_str];
}

// Set target vertex
void AStar::set_trg(const Point &p_trg) {
  // Check target point
  size_t ind_trg = this->map_.pnt_to_ind(p_trg);
  if (ind_trg >= this->map_.n())
    throw "ERROR: Target point is out of map!";
  // Check target box
  if (!this->map_.boxes()[ind_trg].free())
    throw "ERROR: Target box is not free!";
  // Set target vertex
  this->trg_ = &this->vxs_[ind_trg];
}

void AStar::initialize() {
  // Clear open and closed set
  this->open_.clear();
  this->closed_.clear();
  // Init all vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    float h = INF;
    if (this->map_.boxes(ind).free())
      h = this->map_.boxes(ind).cnt().dist(
          this->map_.boxes(this->trg_->ind()).cnt());
    this->vxs_[ind].init(h);
  }
  // Setup start vertex
  this->str_->set_g(0.0f);
  this->str_->set_f(this->str_->h());
  this->open_.insert(this->str_);
}

void AStar::compute_shortest_path() {
  // start search
  ASVx *curr, *neigh;
  float g_score;
  bool in_open, in_closed;
  while (!this->open_.empty()) {
    // Pop fist box in the open set and add it in the closed set
    curr = *this->open_.begin();
    this->open_.erase(this->open_.begin());
    this->closed_.insert(curr->ind());
    // Target box reached
    if (curr->ind() == this->trg_->ind())
      return;
    // Loop on box edges
    for (std::pair<size_t, float> edge :
         this->map_.boxes(curr->ind()).edges()) {
      // Neighbor
      neigh = &this->vxs_[edge.first];
      // Target box reached
      if (neigh->ind() == this->trg_->ind()) {
        this->trg_->set_pred(curr);
        return;
      }
      // Cost to reach the neighbor passing through the current vertex
      g_score = curr->g() + edge.second;
      // Check if vertex is in the open set
      in_open = this->open_.find(neigh) != this->open_.end();
      // Check if vertex is in the closed set
      in_closed = this->closed_.find(neigh->ind()) != this->closed_.end();
      // If it is reached for the first time, it's added to open set
      if (!in_open && !in_closed) {
        neigh->set_pred(curr);
        neigh->set_g(g_score);
        neigh->set_f(neigh->g() + neigh->h());
        this->open_.insert(neigh);
      }
      // Compare the new path with the one already found
      else {
        // New path is better
        if (g_score < neigh->g()) {
          if (in_open)
            this->open_.erase(neigh);
          if (in_closed)
            this->closed_.erase(neigh->ind());
          neigh->set_pred(curr);
          neigh->set_g(g_score);
          neigh->set_f(neigh->g() + neigh->h());
          this->open_.insert(neigh);
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

std::list<Box *> *AStar::path(size_t src_ind) {
  auto *path = new std::list<Box *>;
  ASVx *curr = this->trg_;
  while (curr->ind() != src_ind) {
    path->push_front(&this->map_.boxes(curr->ind()));
    curr = curr->get_pred();
    if (curr == NULL)
      throw "ERROR: No path found";
  }
  return path;
}

void AStar::update_map(std::list<Point> slam_pntcloud) {
  this->map_.slam_update(slam_pntcloud);
}

} // namespace nav