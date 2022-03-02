/**
 * @file Explorer.cpp
 * @brief Source file for class Explorer
 * @date 20 February 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Explorer.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize a map
Explorer::Explorer(float xlen, float ylen, size_t nx, size_t ny, float radius,
                   std::list<Point> exp_fix_pntcloud)
    : xlen_(xlen), ylen_(ylen), nx_(nx), ny_(ny), n_(nx * ny), radius_(radius),
      boxes_(n_) {
  // Compute step
  this->xstep_ = nav::round(xlen / (float)nx);
  this->ystep_ = nav::round(ylen / (float)ny);
  // Map size
  std::vector<size_t> size = {this->nx_, this->ny_};
  // Set the size of the surrounding
  int xy_level = (int)ceil((this->radius_ - (this->xstep_ / 2)) / this->xstep_);
  // Divide the space in boxes
  std::vector<size_t> *idxs;
  std::vector<size_t> *neighs;
  for (size_t ind = 0; ind < this->n_; ind++) {
    // Set the box index
    this->boxes_[ind].set_ind(ind);
    // Set the center of the box
    idxs = nav::ind_to_sub_xy(size, ind);
    if (idxs == NULL)
      throw "ind_to_sub() on " + std::to_string(ind) + " failed!";
    float xcnt = nav::round((this->xstep_ * idxs->at(0)) + (this->xstep_ / 2));
    float ycnt = nav::round((this->ystep_ * idxs->at(1)) + (this->ystep_ / 2));
    Point cnt(xcnt, ycnt, 2.0f);
    this->boxes_[ind].set_cnt(cnt);
    // Check if the box is inside the space
    if (((0 <= (xcnt - this->radius_)) &&
         ((xcnt + this->radius_) <= this->xlen_)) &&
        ((0 <= (ycnt - this->radius_)) &&
         ((ycnt + this->radius_) <= this->ylen_))) {
      this->boxes_[ind].set_in();
    } else {
      this->boxes_[ind].set_out();
    }
    // Set box neighbors
    neighs = nav::find_neighs_xy(size, ind, xy_level);
    if (neighs == NULL)
      throw "find_neighs() on " + std::to_string(ind) + " failed!";
    this->boxes_[ind].set_neighs(*neighs);
  }
  // Assign fixed points to the respective boxes
  for (const Point &pnt : exp_fix_pntcloud) {
    size_t ind = this->pnt_to_ind(pnt);
    if (ind < this->n_)
      this->boxes_[ind].add_fix_pnt(pnt);
  }
  // Set fixed obstacles
  for (size_t ind = 0; ind < this->n_; ind++) {
    size_t count = 0;
    for (size_t ind_neigh : this->boxes_[ind].neighs()) {
      for (const Point &pnt : this->boxes_[ind].fix_pnts()) {
        if (this->boxes_[ind].cnt().dist_xy(pnt) <= this->radius_) {
          count++;
        }
        if (count > 0) {
          this->boxes_[ind].set_busy();
          break;
        }
      }
    }
  }
  // Link boxes close to each other
  std::vector<std::pair<size_t, size_t>> *links;
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (!this->boxes_[ind].is_free() || !this->boxes_[ind].is_in())
      continue;
    // Find links
    links = nav::find_links_xy(size, ind);
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

//
size_t Explorer::choose_dir(size_t curr_ind, size_t curr_dir) {
  //
  srand(time(NULL));
  std::vector<WtEdge> rnd_edges = this->boxes_[curr_ind].edges();
  random_shuffle(rnd_edges.begin(), rnd_edges.end());
  size_t new_dir = std::get<2>(rnd_edges[0]);
  /*if (new_dir == curr_dir) {
    new_dir = std::get<2>(rnd_edges[1]);
  }*/
  return new_dir;
}

//
std::vector<size_t> Explorer::generate_path(size_t curr_ind, size_t curr_dir) {
  //
  srand(time(NULL));
  std::vector<size_t> path = {curr_ind};
  size_t cnt = 3 + rand() % 7;
  std::cout << "rnd: " << cnt << std::endl;
  while (1) {
    bool ok = false;
    if (cnt == 0)
      return path;
    for (WtEdge edge : this->boxes_[curr_ind].edges()) {
      if (curr_dir == std::get<2>(edge)) {
        curr_ind = std::get<0>(edge);
        ok = true;
        break;
      }
    }
    if (ok) {
      path.push_back(curr_ind);
      --cnt;
    } else
      return path;
  }

  /*// Check start point
  std::vector<size_t> *curr_idxs = this->pnt_to_sub(str_pnt);
  if (curr_idxs == NULL)
    throw "ERROR: Start point is out of map!";
  // Check start box
  std::vector<size_t> size = {this->nx_, this->ny_};
  size_t curr_ind = sub_to_ind_xy(size, *curr_idxs);
  if (curr_ind >= this->n_)
    throw "ERROR: Start point is out of map!";
  // Check start box
  if (!this->boxes_[curr_ind].is_free() || !this->boxes_[curr_ind].is_in())
    throw "ERROR: Start box is not free!";

  //
  std::vector<size_t> dir_size = {3, 3};
  std::vector<size_t> *dir_idxs = ind_to_sub_xy(dir_size, dir);
  if (dir_idxs == NULL)
    throw "ind_to_sub() on " + std::to_string(dir) + " failed!";

  //
  std::vector<size_t> path = {curr_ind};
  while (1) {
    curr_idxs->at(0) = curr_idxs->at(0) + dir_idxs->at(0) - 1;
    curr_idxs->at(1) = curr_idxs->at(1) + dir_idxs->at(1) - 1;
    curr_ind = sub_to_ind_xy(size, *curr_idxs);
    if (curr_ind < this->n_ && this->boxes(curr_ind).is_free() &&
        this->boxes(curr_ind).is_in())
      path.push_back(curr_ind);
    else
      return path;
  }*/
}

// Compute the index of the corresponding box
std::vector<size_t> *Explorer::pnt_to_sub(const Point &pnt) {
  if ((pnt.x() < 0.0f || pnt.x() > this->xlen_) ||
      (pnt.y() < 0.0f || pnt.y() > this->ylen_))
    return NULL;
  // Point idxs
  std::vector<size_t> *idxs = new std::vector<size_t>(2);
  idxs->at(0) = (int)floor(pnt.x() / this->xstep_);
  idxs->at(1) = (int)floor(pnt.y() / this->ystep_);
  return idxs;
}

// Compute the index of the corresponding box
size_t Explorer::pnt_to_ind(const Point &pnt) {
  // Point idxs
  std::vector<size_t> idxs(2);
  idxs[0] = (int)floor(pnt.x() / xstep_);
  idxs[1] = (int)floor(pnt.y() / ystep_);
  // Map size
  std::vector<size_t> size = {nx_, ny_};
  // Convert idxs to ind
  return nav::sub_to_ind_xy(size, idxs);
}

} // namespace nav
