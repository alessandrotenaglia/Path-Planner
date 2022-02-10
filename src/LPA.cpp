/**
 * @file LPA.cpp
 * @brief Source file for class LPA
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "LPA.h"

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace nav {

LPA::LPA(Map map) : map_(map), vxs_(map.n()), str_(NULL), trg_(NULL) {
  // Loop on map boxes
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    // Set the box index
    this->vxs_[ind].set_ind(ind);
  }
}

// Set start vertex
void LPA::set_str(const Point &p_str) {
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
void LPA::set_trg(const Point &p_trg) {
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

void LPA::initialize() {
  // Clear open set
  this->open_.clear();
  // Init all vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    float h = INF;
    if (this->map_.boxes(ind).free())
      h = this->map_.boxes(ind).cnt().dist(
          this->map_.boxes(this->trg_->ind()).cnt());
    this->vxs_[ind].init(h);
  }
  // Setup start vertex
  this->str_->set_rhs(0.0f);
  this->str_->calculate_key();
  this->open_.insert(this->str_);
}

void LPA::update_vx(LPAVx *vx) {
  if (vx->ind() != this->str_->ind()) {
    // Check if vertex is in the open set
    if (this->open_.find(vx) != this->open_.end())
      this->open_.erase(vx);
    //
    if (!this->map_.boxes(vx->ind()).free()) {
      vx->set_g(INF);
      vx->set_h(INF);
      vx->set_rhs(INF);
    } else {
      //
      float min_rhs = INF;
      for (WtEdge edge : this->map_.boxes(vx->ind()).edges()) {
        if (!this->map_.boxes(edge.first).free()) {
        }
        min_rhs = std::min(min_rhs, this->vxs_[edge.first].g() + edge.second);
      }
      vx->set_rhs(min_rhs);
      //
      if (vx->g() != vx->rhs()) {
        vx->calculate_key();
        this->open_.insert(vx);
      }
    }
  }
}

void LPA::compute_shortest_path() {
  LPAVx *curr;
  while ((*this->open_.begin() < this->trg_) ||
         (this->trg_->rhs() != this->trg_->g())) {
    // Pop fist vertex in the open set
    curr = *this->open_.begin();
    this->open_.erase(this->open_.begin());
    // Check if the vertex is over-consistent
    if (curr->g() > curr->rhs()) {
      curr->set_g(curr->rhs());
      for (WtEdge edge : this->map_.boxes(curr->ind()).edges())
        this->update_vx(&this->vxs_[edge.first]);
    } else {
      curr->set_g(INF);
      for (WtEdge edge : this->map_.boxes(curr->ind()).edges())
        this->update_vx(&this->vxs_[edge.first]);
      this->update_vx(curr);
    }
  }
}

std::list<Box *> *LPA::path(size_t src_ind) {
  if (this->trg_->g() == INF)
    throw "ERROR: No path found";
  auto *path = new std::list<Box *>;
  LPAVx *curr = this->trg_;
  path->push_front(&this->map_.boxes(curr->ind()));
  while (curr->ind() != src_ind) {
    float score, min_val = INF;
    size_t min_ind = -1;
    for (std::pair<size_t, float> edge :
         this->map_.boxes(curr->ind()).edges()) {
      score = this->vxs_[edge.first].g() + edge.second;
      if (score < min_val) {
        min_val = score;
        min_ind = edge.first;
      }
    }
    if (min_ind == src_ind)
      break;
    curr = &this->vxs_[min_ind];
    path->push_front(&this->map_.boxes(min_ind));
  }
  return path;
}

void LPA::update_map(std::list<Point> slam_pntcloud) {
  this->map_.slam_update(slam_pntcloud);
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    if (this->map_.updated(ind)) {
      this->update_vx(&this->vxs_[ind]);
    }
  }
}

} // namespace nav
