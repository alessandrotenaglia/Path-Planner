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
#include "FibonacciHeap.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize A*
AStar::AStar(Map map) : map_(map), vxs_(map.n()), str_(-1), trg_(-1) {
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
  if (!this->map_.boxes()[str_ind].is_free() ||
      !this->map_.boxes()[str_ind].is_in())
    throw "ERROR: Start box is not free!";
  // Set start vertex
  this->str_ = str_ind;
}

// Set target vertex
void AStar::set_trg(const Point &trg_pnt) {
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
    this->vxs_[ind].set_h(
        this->map_.boxes(ind).cnt().dist(this->map_.boxes(this->trg_).cnt()));
}

// Compute shortest path
void AStar::compute_shortest_path() {
  // Initialize vertices
  for (size_t ind = 0; ind < this->vxs_.size(); ind++) {
    this->vxs_[ind].set_g(INF);
    if (!this->map_.boxes(this->vxs_[ind].ind()).is_free() ||
        !this->map_.boxes(this->vxs_[ind].ind()).is_in()) {
      this->vxs_[ind].set_h(INF);
    }
    this->vxs_[ind].set_f();
    this->vxs_[ind].set_pred(-1);
  }
  // Initialize OPEN and close set
  FibonacciHeap<ASNode> OPEN;
  std::unordered_set<size_t> CLOSED;
  // Setup start vertex
  this->vxs_[this->str_].set_g(0.0f);
  this->vxs_[this->str_].set_f();
  OPEN.insert(ASNode(this->str_, this->vxs_[this->str_].f()));
  // Loop on OPEN set
  while (!OPEN.isEmpty()) {
    // Pop first vertex from the OPEN set and add it to the CLOSED set
    ASNode curr = OPEN.removeMinimum();
    CLOSED.insert(curr.ind());
    // Check if the target has been reached
    if (curr.ind() == this->trg_) {
      this->set_path();
      return;
    }
    // Loop on edges
    for (WtEdge edge : this->map_.boxes(curr.ind()).edges()) {
      // Cost to reach the link passing through the current vertex
      float g_score = util::round(this->vxs_[curr.ind()].g() + edge.second);
      // Check if the vertex is in the OPEN set
      node<ASNode> *temp =
          OPEN.find(ASNode(edge.first, this->vxs_[edge.first].f()));
      bool in_OPEN = (temp != NULL);
      // Check if the vertex is in the CLOSED set
      bool in_CLOSED = (CLOSED.find(edge.first) != CLOSED.end());
      //
      if (!in_OPEN && !in_CLOSED) {
        this->vxs_[edge.first].set_g(g_score);
        this->vxs_[edge.first].set_f();
        this->vxs_[edge.first].set_pred(curr.ind());
        OPEN.insert(ASNode(edge.first, this->vxs_[edge.first].f()));
      } else {
        if (g_score < this->vxs_[edge.first].g()) {
          this->vxs_[edge.first].set_g(g_score);
          this->vxs_[edge.first].set_f();
          this->vxs_[edge.first].set_pred(curr.ind());
          if (in_OPEN) {
            OPEN.decreaseKey(temp,
                             ASNode(edge.first, this->vxs_[edge.first].f()));
          }
          if (in_CLOSED) {
            CLOSED.erase(edge.first);
            OPEN.insert(ASNode(edge.first, this->vxs_[edge.first].f()));
          }
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

// Set path
void AStar::set_path() {
  this->path_.clear();
  size_t ind = this->trg_;
  while (ind != this->str_) {
    this->path_.push_front(ind);
    ind = this->vxs_[ind].pred();
  }
}

// Update map with SLAM pointcloud,
void AStar::update(std::list<Point> slam_pntcloud) {
  std::list<size_t> updated = this->map_.slam_update(slam_pntcloud);
  bool flag = false;
  for (size_t ind : this->path_)
    if (!this->map().boxes(ind).is_free() || !this->map().boxes(ind).is_in()) {
      flag = true;
      break;
    }
  if (flag) {
    this->compute_shortest_path();
  }
}

} // namespace nav
