/**
 * @file Map.cpp
 * @brief Source file for class Map
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

#define INF std::numeric_limits<float>::max();

// Initialize a map
Map::Map(float xlen, float ylen, float zlen, size_t nx, size_t ny, size_t nz,
         float radius, float height, std::list<Point> fix_pntcloud)
    : xlen_(xlen), ylen_(ylen), zlen_(zlen), nx_(nx), ny_(ny), nz_(nz),
      n_(nx * ny * nz), radius_(radius), height_(height), boxes_(n_),
      updatable_(n_, true) {
  //
  this->xstep_ = util::round(xlen / (float)nx);
  this->ystep_ = util::round(ylen / (float)ny);
  this->zstep_ = util::round(zlen / (float)nz);
  // Map size
  std::vector<size_t> size = {this->nx_, this->ny_, this->nz_};
  // Set the size of the surrounding
  int xy_level = (int)ceil((this->radius_ - this->xstep_ / 2) / this->xstep_);
  int z_level = (int)ceil((this->height_ - this->zstep_ / 2) / this->zstep_);
  // Divide the space in boxes
  std::vector<size_t> *idxs;
  std::vector<size_t> *neighs;
  for (size_t ind = 0; ind < this->n_; ind++) {
    // Set the box index
    this->boxes_[ind].set_ind(ind);
    // Set the center of the box
    idxs = util::ind_to_sub(size, ind);
    if (idxs == NULL)
      throw "ind_to_sub() on " + std::to_string(ind) + " failed!";
    float xcnt = util::round((this->xstep_ * idxs->at(0)) + this->xstep_ / 2);
    float ycnt = util::round((this->ystep_ * idxs->at(1)) + this->ystep_ / 2);
    float zcnt = util::round((this->zstep_ * idxs->at(2)) + this->zstep_ / 2);
    Point cnt(xcnt, ycnt, zcnt);
    this->boxes_[ind].set_cnt(cnt);
    // Check if the box is inside the space
    if ((0 <= cnt.x() - this->radius_ && cnt.x() + this->radius_ <= xlen_) &&
        (0 <= cnt.y() - this->radius_ && cnt.y() + this->radius_ <= ylen_) &&
        (0 <= cnt.z() - this->height_ && cnt.z() + this->height_ <= zlen_))
      this->boxes_[ind].set_in(true);
    // Set box neighbors
    neighs = util::find_neighs(size, ind, xy_level, z_level);
    if (neighs == NULL)
      throw "find_neighs() on " + std::to_string(ind) + " failed!";
    this->boxes_[ind].set_neighs(*neighs);
  }
  // Assign fixed points to the respective boxes
  for (const Point &pnt : fix_pntcloud) {
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
            this->boxes_[ind].cnt().dist_z(pnt) <= this->height_)
          count++;
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
  std::vector<size_t> *links;
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (!this->boxes_[ind].free() || !this->boxes_[ind].in())
      continue;
    // Find links
    links = util::find_links(size, ind);
    if (links == NULL)
      throw "find_links() on " + std::to_string(ind) + " failed!";
    // Set links
    for (size_t link : *links) {
      if (this->boxes_[link].free() && this->boxes_[link].in()) {
        this->boxes_[ind].add_edge(
            link, this->boxes_[ind].cnt().dist(this->boxes_[link].cnt()));
      }
    }
  }
}

// Update map from SLAM pointcloud
std::list<size_t> Map::slam_update(std::list<Point> slam_pntcloud) {
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
            if (this->boxes_[ind].free())
              updated.push_back(ind);
            this->boxes_[ind].set_busy();
            break;
          }
        }
        if (count > 0)
          break;
      }
    }
  }
  return updated;
}

// Compute the index of the corresponding box
size_t Map::pnt_to_ind(const Point &pnt) {
  // Point idxs
  std::vector<size_t> idxs(3);
  idxs[0] = (int)floor(pnt.x() / xstep_);
  idxs[1] = (int)floor(pnt.y() / ystep_);
  idxs[2] = (int)floor(pnt.z() / zstep_);
  // Map size
  std::vector<size_t> size = {nx_, ny_, nz_};
  // Convert idxs to ind
  return util::sub_to_ind(size, idxs);
}

} // namespace nav
