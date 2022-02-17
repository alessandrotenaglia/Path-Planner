/**
 * @file LPAStar.cpp
 * @brief Source file for class LPAStar
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "LPAStar.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize A*
LPAStar::LPAStar(Map map) : map_(map), vxs_(map.n()), str_(-1), trg_(-1) {
  // Loop on vertices
  for (size_t ind = 0; ind < this->map_.n(); ind++) {
    // Set the box index
    this->vxs_[ind].set_ind(ind);
  }
}

// Set start vertex from point
void LPAStar::set_str(const Point &str_pnt) {
  this->set_str(this->map_.pnt_to_ind(str_pnt));
}

// Set start vertex from index
void LPAStar::set_str(size_t str_ind) {
  // Check start index
  if (str_ind >= this->map_.n())
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->map_.boxes()[str_ind].is_free() ||
      !this->map_.boxes()[str_ind].is_in())
    throw "ERROR: Start box is not free!";
  // Set start vertex
  this->str_ = str_ind;
}

// Set target vertex
void LPAStar::set_trg(const Point &trg_pnt) {
  // Check target point
  size_t trg_ind = this->map_.pnt_to_ind(trg_pnt);
  if (trg_ind >= this->map_.n())
    throw "ERROR: Target point is out of map!";
  // Check target box
  if (!this->map_.boxes()[trg_ind].is_free() ||
      !this->map_.boxes()[trg_ind].is_in())
    throw "ERROR: Target box is not free!";
  // Set target vertex
  this->trg_ = trg_ind;
  // Initialize vertices
  for (size_t ind = 0; ind < this->vxs_.size(); ind++)
    if (this->map_.updatable(ind)) {
      this->vxs_[ind].set_h(
          this->map_.boxes(ind).cnt().dist(this->map_.boxes(this->trg_).cnt()));
    } else {
      this->vxs_[ind].set_h(INF);
    }
}

// Initialize
void LPAStar::initialize() {
  this->OPEN = FibonacciHeap<LPANode>();
  for (size_t ind = 0; ind < this->vxs_.size(); ind++) {
    this->vxs_[ind].set_g(INF);
    this->vxs_[ind].set_rhs(INF);
  }
  this->vxs_[this->str_].set_rhs(0.0f);
  this->vxs_[this->str_].calculate_key();
  this->OPEN.insert(LPANode(this->str_, this->vxs_[this->str_].k1(),
                            this->vxs_[this->str_].k2()));
}

// Compute shortest path
void LPAStar::compute_shortest_path() {
  while ((this->OPEN.getMinimum() < this->vxs_[this->trg_].node()) ||
         (this->vxs_[this->trg_].g() != this->vxs_[this->trg_].rhs())) {
    LPANode curr = OPEN.removeMinimum();
    if (this->vxs_[curr.ind()].g() > this->vxs_[curr.ind()].rhs()) {
      this->vxs_[curr.ind()].set_g(this->vxs_[curr.ind()].rhs());
    } else {
      this->vxs_[curr.ind()].set_g(INF);
      this->update_node(curr.ind());
    }
    for (WtEdge edge : this->map_.boxes(curr.ind()).edges()) {
      this->update_node(edge.first);
    }
  }
}

void LPAStar::update_node(size_t ind) {
  if (ind != this->str_) {
    node<LPANode> *temp = OPEN.find(this->vxs_[ind].node());
    float rhs_score = INF;
    for (WtEdge edge : this->map_.boxes(ind).edges()) {
      rhs_score = std::min(rhs_score, this->vxs_[edge.first].rhs());
    }
    this->vxs_[ind].set_rhs(rhs_score);
    if (this->vxs_[ind].g() == this->vxs_[ind].rhs()) {
      if (temp != NULL) {
        OPEN.decreaseKey(temp, LPANode(ind, 0.0f, 0.0f));
        OPEN.removeMinimum();
      }
    } else {
      OPEN.decreaseKey(temp, this->vxs_[ind].node());
    }
  }
}

// Set path
std::list<size_t> LPAStar::path(size_t curr_ind) {
  if (this->vxs_[this->trg_].g() == INF)
    throw "ERROR: No path found";
  std::list<size_t> path;
  size_t ind = this->trg_;
  while (ind != this->str_) {
    float score, min_val = INF;
    size_t min_ind = -1;
    for (WtEdge edge : this->map_.boxes(ind).edges()) {
      score = this->vxs_[edge.first].g() + edge.second;
      if (score < min_val) {
        min_val = score;
        min_ind = edge.first;
      }
    }
    if (min_ind == curr_ind)
      break;
    ind = min_ind;
    path.push_front(min_ind);
  }
  return path;
}

// Update map with SLAM pointcloud,
void LPAStar::update_map(std::list<Point> slam_pntcloud) {
  std::list<size_t> updated = this->map_.slam_update(slam_pntcloud);
  for (size_t ind : updated) {
    this->update_node(ind);
  }
}

} // namespace nav
