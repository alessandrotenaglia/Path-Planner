/**
 * @file Planner.cpp
 * @brief Source file for class Planner
 * @date 20 February 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Planner.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize a map
Planner::Planner(float xlen, float ylen, float zlen, size_t nx, size_t ny,
                 size_t nz, float radius, float height,
                 std::list<Point> nav_fix_pntcloud)
    : xlen_(xlen), ylen_(ylen), zlen_(zlen), nx_(nx), ny_(ny), nz_(nz),
      n_(nx * ny * nz), radius_(radius), height_(height), boxes_(n_),
      updatable_(n_, true) {
  // Compute step
  this->xstep_ = nav::round(xlen / (float)nx);
  this->ystep_ = nav::round(ylen / (float)ny);
  this->zstep_ = nav::round(zlen / (float)nz);
  // Map size
  std::vector<size_t> size = {this->nx_, this->ny_, this->nz_};
  // Set the size of the surrounding
  int xy_level = (int)ceil((this->radius_ - (this->xstep_ / 2)) / this->xstep_);
  int z_level = (int)ceil((this->height_ - (this->zstep_ / 2)) / this->zstep_);
  // Divide the space in boxes
  std::vector<size_t> *idxs;
  std::vector<size_t> *neighs;
  for (size_t ind = 0; ind < this->n_; ind++) {
    // Set the box index
    this->boxes_[ind].set_ind(ind);
    // Set the center of the box
    idxs = nav::ind_to_sub(size, ind);
    if (idxs == NULL)
      throw "ind_to_sub() on " + std::to_string(ind) + " failed!";
    float xcnt = nav::round((this->xstep_ * idxs->at(0)) + (this->xstep_ / 2));
    float ycnt = nav::round((this->ystep_ * idxs->at(1)) + (this->ystep_ / 2));
    float zcnt = nav::round((this->zstep_ * idxs->at(2)) + (this->zstep_ / 2));
    Point cnt(xcnt, ycnt, zcnt);
    this->boxes_[ind].set_cnt(cnt);
    // Check if the box is inside the space
    if (((0 <= (xcnt - this->radius_)) &&
         ((xcnt + this->radius_) <= this->xlen_)) &&
        ((0 <= (ycnt - this->radius_)) &&
         ((ycnt + this->radius_) <= this->ylen_)) &&
        ((0 <= (zcnt - this->height_)) &&
         ((zcnt + this->height_) <= this->zlen_))) {
      this->boxes_[ind].set_in();
    } else {
      this->boxes_[ind].set_out();
      this->updatable_[ind] = false;
    }
    // Set box neighbors
    neighs = nav::find_neighs(size, ind, xy_level, z_level);
    if (neighs == NULL)
      throw "find_neighs() on " + std::to_string(ind) + " failed!";
    this->boxes_[ind].set_neighs(*neighs);
  }
  // Assign fixed points to the respective boxes
  for (const Point &pnt : nav_fix_pntcloud) {
    size_t ind = this->pnt_to_ind(pnt);
    if (ind < this->n_)
      this->boxes_[ind].add_fix_pnt(pnt);
  }
  // Set fixed obstacles
  for (size_t ind = 0; ind < this->n_; ind++) {
    size_t count = 0;
    for (size_t ind_neigh : this->boxes_[ind].neighs()) {
      for (const Point &pnt : this->boxes_[ind_neigh].fix_pnts()) {
        if (this->boxes_[ind].cnt().dist_xy(pnt) <= this->radius_ &&
            this->boxes_[ind].cnt().dist_z(pnt) <= this->height_) {
          count++;
        }
        if (count > 0) {
          this->boxes_[ind].set_busy();
          this->updatable_[ind] = false;
          break;
        }
      }
      if (count > 0)
        break;
    }
  }
  // Link boxes close to each other
  std::vector<std::pair<size_t, size_t>> *links;
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (!this->boxes_[ind].is_free() || !this->boxes_[ind].is_in())
      continue;
    // Find links
    links = nav::find_links(size, ind);
    if (links == NULL)
      throw "find_links() on " + std::to_string(ind) + " failed!";
    // Set links
    for (std::pair<size_t, size_t> link : *links) {
      if (this->boxes_[link.first].is_free() &&
          this->boxes_[link.first].is_in()) {
        this->boxes_[ind].add_edge(
            link.first,
            this->boxes_[ind].cnt().dist(this->boxes_[link.first].cnt()),
            link.second);
      }
    }
  }
}

// Set start box
void Planner::set_str(const Point &str_pnt) {
  // Check start point
  size_t str_ind = this->pnt_to_ind(str_pnt);
  if (str_ind >= this->n_)
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->boxes_[str_ind].is_free() || !this->boxes_[str_ind].is_in())
    throw "ERROR: Start box is not free!";
  // Set start box
  this->str_ = str_ind;
}

// Set target box
void Planner::set_trg(const Point &trg_pnt) {
  // Check target point
  size_t trg_ind = this->pnt_to_ind(trg_pnt);
  if (trg_ind >= this->n_)
    throw "ERROR: Target point is out of map!";
  // Check target box
  if (!this->boxes_[trg_ind].is_free() || !this->boxes_[trg_ind].is_in())
    throw "ERROR: Target box is not free!";
  // Set target box
  this->trg_ = trg_ind;
  // Initialize boxes
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (this->updatable_[ind]) {
      this->boxes_[ind].set_h(
          this->boxes_[ind].cnt().dist(this->boxes_[this->trg_].cnt()));
    } else {
      this->boxes_[ind].set_h(INF);
    }
  }
}

// Compute shortest path
void Planner::search() {
  // Initialize OPEN and close set
  FibonacciHeap<Node> OPEN;
  std::unordered_set<size_t> CLOSED;
  // Setup start box
  this->boxes_[this->str_].set_g(0.0f);
  OPEN.insert(Node(this->str_, this->boxes_[this->str_].get_f()));
  // Loop on OPEN set
  while (!OPEN.isEmpty()) {
    // Pop first vertex from the OPEN set and add it to the CLOSED set
    Node curr = OPEN.removeMinimum();
    CLOSED.insert(curr.ind());
    // Check if the target has been reached
    if (curr.ind() == this->trg_) {
      this->set_path();
      return;
    }
    // Loop on edges
    for (WtEdge edge : this->boxes_[curr.ind()].edges()) {
      // Cost to reach the link passing through the current vertex
      float g_score =
          nav::round(this->boxes_[curr.ind()].g() + std::get<1>(edge));
      // Check if the vertex is in the OPEN set
      node<Node> *temp = OPEN.find(
          Node(std::get<0>(edge), this->boxes_[std::get<0>(edge)].get_f()));
      bool in_OPEN = (temp != NULL);
      // Check if the vertex is in the CLOSED set
      bool in_CLOSED = (CLOSED.find(std::get<0>(edge)) != CLOSED.end());
      //
      if (!in_OPEN && !in_CLOSED) {
        this->boxes_[std::get<0>(edge)].set_g(g_score);
        this->boxes_[std::get<0>(edge)].set_pred(curr.ind());
        OPEN.insert(
            Node(std::get<0>(edge), this->boxes_[std::get<0>(edge)].get_f()));
      } else {
        if (g_score < this->boxes_[std::get<0>(edge)].g()) {
          this->boxes_[std::get<0>(edge)].set_g(g_score);
          this->boxes_[std::get<0>(edge)].set_pred(curr.ind());
          if (in_OPEN) {
            OPEN.decreaseKey(temp,
                             Node(std::get<0>(edge),
                                  this->boxes_[std::get<0>(edge)].get_f()));
          }
          if (in_CLOSED) {
            CLOSED.erase(std::get<0>(edge));
            OPEN.insert(Node(std::get<0>(edge),
                             this->boxes_[std::get<0>(edge)].get_f()));
          }
        }
      }
    }
  }
  throw "ERROR: No path found!";
}

// Set path
void Planner::set_path() {
  // Clear old path
  this->path_.clear();
  size_t ind = this->trg_;
  while (ind != this->str_) {
    this->path_.push_front(ind);
    ind = this->boxes_[ind].pred();
    if (ind == -1) {
      throw "ERROR: No path found!";
    }
  }
}

// Update map from SLAM pointcloud
void Planner::update(std::list<Point> slam_pntcloud) {
  // Assign SLAM points to the respective boxes
  std::vector<bool> toverify(this->n_, false);
  for (const Point &pnt : slam_pntcloud) {
    size_t ind = this->pnt_to_ind(pnt);
    if (ind < this->n_ && this->updatable_[ind]) {
      this->boxes_[ind].add_slam_pnt(pnt);
      for (size_t ind_neigh : this->boxes_[ind].neighs()) {
        toverify[ind_neigh] = this->updatable_[ind_neigh];
      }
    }
  }
  // Set SLAM obsatcles
  std::list<size_t> updated;
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (toverify[ind]) {
      size_t count = 0;
      for (size_t ind_neigh : this->boxes_[ind].neighs()) {
        for (const Point &pnt : this->boxes_[ind_neigh].slam_pnts()) {
          if (this->boxes_[ind].cnt().dist_xy(pnt) <= this->radius_ &&
              this->boxes_[ind].cnt().dist_z(pnt) <= this->height_)
            count++;
          if (count > 0) {
            if (this->boxes_[ind].is_free())
              updated.push_back(ind);
            this->boxes_[ind].set_busy();
            this->boxes_[ind].set_h(INF);
            break;
          }
        }
        if (count > 0)
          break;
      }
    }
  }
  // Check if there are obstacles along the path
  for (size_t ind : this->path_) {
    if (!this->boxes_[ind].is_free() || !this->boxes_[ind].is_in()) {
      this->search();
      return;
    }
  }
}

//
size_t Planner::move() {
  this->str_ = this->path_.front();
  this->path_.pop_front();
  return this->str_;
}

// Compute the index of the corresponding box
size_t Planner::pnt_to_ind(const Point &pnt) {
  // Point idxs
  std::vector<size_t> idxs(3);
  idxs[0] = (int)floor(pnt.x() / xstep_);
  idxs[1] = (int)floor(pnt.y() / ystep_);
  idxs[2] = (int)floor(pnt.z() / zstep_);
  // Map size
  std::vector<size_t> size = {nx_, ny_, nz_};
  // Convert idxs to ind
  return nav::sub_to_ind(size, idxs);
}

} // namespace nav
