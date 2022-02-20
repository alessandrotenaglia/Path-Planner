/**
 * @file ExpMap.cpp
 * @brief Source file for class ExpMap
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "ExpMap.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace nav {

// Initialize a map
ExpMap::ExpMap(float xlen, float ylen, size_t nx, size_t ny, float radius,
               std::list<Point> exp_fix_pntcloud)
    : xlen_(xlen), ylen_(ylen), nx_(nx), ny_(ny), n_(nx * ny), radius_(radius),
      boxes_(n_) {
  // Compute step
  this->xstep_ = nav::round(xlen / (float)nx);
  this->ystep_ = nav::round(ylen / (float)ny);
  // Map size
  std::vector<size_t> size = {this->nx_, this->ny_};
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
  // Link boxes close to each other
  std::vector<size_t> *links;
  for (size_t ind = 0; ind < this->n_; ind++) {
    if (!this->boxes_[ind].is_free())
      continue;
    // Find links
    links = nav::find_links_xy(size, ind);
    if (links == NULL)
      throw "find_links() on " + std::to_string(ind) + " failed!";
    // Set links
    size_t f = 0;
    for (size_t link : *links) {
      if (this->boxes_[link].is_free()) {
        this->boxes_[ind].add_edge(
            link, this->boxes_[ind].cnt().dist(this->boxes_[link].cnt()));
        f++;
      }
      this->boxes_[ind].set_f(f);
    }
  }
}

// Compute the index of the corresponding box
size_t ExpMap::pnt_to_ind(const Point &pnt) {
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
