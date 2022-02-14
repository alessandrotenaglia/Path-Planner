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
  if (!this->map_.boxes()[str_ind].free() || !this->map_.boxes()[str_ind].in())
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
  if (!this->map_.boxes()[trg_ind].free() || !this->map_.boxes()[trg_ind].in())
    throw "ERROR: Target box is not free!";
  // Set target vertex
  this->trg_ = trg_ind;
}

// Get path
std::list<const Box *> *AStar::path(size_t curr_ind) {
  auto *path = new std::list<const Box *>;
  size_t ind = this->trg_;
  while (ind != curr_ind) {
    path->push_front(&this->map_.boxes(ind));
    ind = this->vxs_[ind].pred();
    if (ind == -1)
      throw "ERROR: No path found";
  }
  return path;
}

void AStar::compute_shortest_path() {
  // Initialize vertices
  for (size_t ind = 0; ind < this->vxs_.size(); ind++) {
    this->vxs_[ind].set_g(INF);
    if (this->map_.boxes(ind).free() && this->map_.boxes(ind).in()) {
      this->vxs_[ind].set_h(
          this->map_.boxes(ind).cnt().dist(this->map_.boxes(this->trg_).cnt()));
    } else {
      this->vxs_[ind].set_h(INF);
    }
    this->vxs_[ind].set_f();
    this->vxs_[ind].set_pred(-1);
  }
  // Initialize OPEN and close set
  std::vector<size_t> OPEN;
  std::vector<size_t> CLOSED;
  // Setup start vertex
  this->vxs_[this->str_].set_g(0.0f);
  this->vxs_[this->str_].set_f();
  OPEN.push_back(this->str_);
  // Loop on OPEN set
  while (!OPEN.empty()) {
    float min_f = INF;
    size_t min_OPEN;
    for (size_t ind = 0; ind < OPEN.size(); ind++) {
      if (this->vxs_[OPEN[ind]].f() < min_f) {
        min_f = this->vxs_[OPEN[ind]].f();
        min_OPEN = ind;
      }
    }
    // Pop first vertex from the OPEN set and add it to the CLOSED set
    size_t curr_ind = OPEN[min_OPEN];
    OPEN.erase(OPEN.begin() + min_OPEN);
    CLOSED.push_back(curr_ind);
    // Check if the target has been reached
    if (curr_ind == this->trg_) {
      return;
    }
    // Loop on edges
    for (WtEdge edge : this->map_.boxes(this->vxs_[curr_ind].ind()).edges()) {
      // Cost to reach the link passing through the current vertex
      float g_score = util::round(this->vxs_[curr_ind].g() + edge.second);
      // Check if the vertex is in the OPEN set
      size_t OPEN_ind;
      bool in_OPEN = false;
      for (size_t ind = 0; ind < OPEN.size(); ind++) {
        if (OPEN[ind] == edge.first) {
          OPEN_ind = ind;
          in_OPEN = true;
          break;
        }
      }
      // Check if the vertex is in the CLOSED set
      size_t CLOSED_ind;
      bool in_CLOSED = false;
      for (size_t ind = 0; ind < CLOSED.size(); ind++) {
        if (CLOSED[ind] == edge.first) {
          CLOSED_ind = ind;
          in_CLOSED = true;
          break;
        }
      }
      // Check if vertex has already been visited
      if (!in_OPEN && !in_CLOSED) {
        // It's visited for the first time, so it's added to OPEN set
        this->vxs_[edge.first].set_g(g_score);
        this->vxs_[edge.first].set_f();
        this->vxs_[edge.first].set_pred(curr_ind);
        OPEN.push_back(edge.first);
      }
      // Compare the new path with the one already found
      else {
        // New path is better
        if (g_score < this->vxs_[edge.first].g()) {
          if (in_OPEN)
            OPEN.erase(OPEN.begin() + OPEN_ind);
          if (in_CLOSED)
            CLOSED.erase(CLOSED.begin() + CLOSED_ind);
          this->vxs_[edge.first].set_g(g_score);
          this->vxs_[edge.first].set_f();
          this->vxs_[edge.first].set_pred(curr_ind);
          OPEN.push_back(edge.first);
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

void AStar::update(size_t curr_ind, std::list<Point> slam_pntcloud) {
  if (this->map_.slam_update(slam_pntcloud)) {
    this->set_str(curr_ind);
    this->compute_shortest_path();
  }
}

} // namespace nav