/**
 * @file generate_pntcloud.h
 * @brief Source file for the generation of the pointclouds
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/list.hpp>
#include <fstream>
#include <list>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Point.h"

/*---------------------------------------------------------------------------*/
/*                              Main Definition                              */
/*---------------------------------------------------------------------------*/

#define _A 0.5
#define _B 1.5
#define _C 2.5
#define _D 3.5
#define _E 4.5
#define _F 5.5
#define _G 6.5
#define _H 7.5
#define _I 8.5
#define _L 9.5

#define _1 19.5
#define _2 18.5
#define _3 17.5
#define _4 16.5
#define _5 15.5
#define _6 14.5
#define _7 13.5
#define _8 12.5
#define _9 11.5
#define _10 10.5
#define _11 9.5
#define _12 8.5
#define _13 7.5
#define _14 6.5
#define _15 5.5
#define _16 4.5
#define _17 3.5
#define _18 2.5
#define _19 1.5
#define _20 0.5

std::pair<nav::Point, nav::Point> make_obstacle(float x, float y) {
  nav::Point O1(x - 0.25, y - 0.25, 0.0);
  nav::Point O2(x + 0.25, y + 0.25, 3.0);
  return std::make_pair(O1, O2);
}

int main() {
  std::cout << "Il godo..." << std::endl;

  std::list<std::pair<nav::Point, nav::Point>> nav_fix_obstacles;
  std::list<std::pair<nav::Point, nav::Point>> exp_fix_obstacles;

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(0.0, 0.0, 0.0), nav::Point(1.0, 3.0, 1.9)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(0.0, 0.0, 0.0), nav::Point(1.0, 3.0, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 2.0, 0.0), nav::Point(0.0, 1.0, 1.9)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 2.0, 0.0), nav::Point(0.0, 1.0, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 6.0, 0.0), nav::Point(3.3, 6.7, 1.2)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 6.7, 0.0), nav::Point(1.6, 7.2, 1.2)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(2.7, 6.7, 0.0), nav::Point(3.2, 7.2, 1.2)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 7.9, 0.0), nav::Point(1.6, 8.4, 1.2)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(2.7, 7.9, 0.0), nav::Point(3.2, 8.4, 1.2)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 8.4, 0.0), nav::Point(3.3, 9.1, 1.2)));
  /*exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(1.0, 6.0, 0.0), nav::Point(3.3, 9.1, 3.0)));*/

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(4.8, 7.1, 0.0), nav::Point(5.5, 8.0, 1.6)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(4.8, 7.1, 0.0), nav::Point(5.5, 8.0, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(5.5, 6.6, 0.0), nav::Point(6.0, 8.4, 1.6)));
  /*exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(5.5, 6.6, 0.0), nav::Point(6.0, 8.4, 3.0)));*/
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(0.0, 6.0, 0.0), nav::Point(6.0, 10.0, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 0.0, 0.0), nav::Point(8.8, 1.2, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 1.2, 2.5), nav::Point(8.8, 2.6, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 2.6, 0.0), nav::Point(8.8, 3.8, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.8, 0.0, 2.5), nav::Point(10.0, 0.8, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.8, 3.0, 2.5), nav::Point(10.0, 3.8, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(10.0, 0.0, 0.0), nav::Point(10.7, 1.2, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(10.0, 1.2, 2.5), nav::Point(10.7, 2.6, 3.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(10.0, 2.6, 0.0), nav::Point(10.7, 3.8, 3.0)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 0.0, 0.0), nav::Point(10.7, 3.8, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 5.8, 0.0), nav::Point(10.5, 7.2, 2.0)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(8.0, 5.8, 0.0), nav::Point(10.5, 7.2, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(11.0, 8.1, 0.0), nav::Point(15.7, 9.8, 0.7)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(12.7, 9.0, 0.7), nav::Point(15.7, 9.8, 2.0)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(14.8, 8.1, 0.7), nav::Point(15.7, 9.0, 2.0)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(12.7, 8.1, 0.0), nav::Point(15.7, 9.8, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(13.5, 0.0, 0.0), nav::Point(15.9, 1.8, 0.7)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(13.5, 0.0, 0.7), nav::Point(14.6, 1.3, 1.9)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(15.9, 0.4, 0.0), nav::Point(17.0, 1.3, 1.4)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(13.5, 0.0, 0.0), nav::Point(17.0, 1.8, 3.0)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(16.0, 3.0, 0.0), nav::Point(16.7, 4.1, 0.9)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(16.0, 5.3, 0.0), nav::Point(17.1, 6.0, 0.9)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(17.9, 3.0, 0.0), nav::Point(19.0, 3.7, 0.9)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(18.3, 4.9, 0.0), nav::Point(19.0, 6.0, 0.9)));

  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(17.8, 7.8, 0.0), nav::Point(20.0, 10.0, 0.8)));
  nav_fix_obstacles.push_back(
      std::make_pair(nav::Point(17.8, 7.8, 0.8), nav::Point(19.0, 9.0, 2.0)));
  exp_fix_obstacles.push_back(
      std::make_pair(nav::Point(17.8, 7.8, 0.0), nav::Point(20.0, 10.0, 3.0)));

  std::list<nav::Point> nav_fix_pntcloud;
  for (std::pair<nav::Point, nav::Point> &obs : nav_fix_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          nav_fix_pntcloud.push_back(pnt);
        }
      }
    }
  }
  {
    std::ofstream ofs("../data/nav_fix_pntcloud.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << nav_fix_pntcloud;
  }

  std::list<nav::Point> exp_fix_pntcloud;
  for (std::pair<nav::Point, nav::Point> &obs : exp_fix_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          exp_fix_pntcloud.push_back(pnt);
        }
      }
    }
  }
  {
    std::ofstream ofs("../data/exp_fix_pntcloud.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << exp_fix_pntcloud;
  }

  std::list<std::pair<nav::Point, nav::Point>> slam_obstacles;

  slam_obstacles.push_back(make_obstacle(_7, _E));
  slam_obstacles.push_back(make_obstacle(_11, _E));
  slam_obstacles.push_back(make_obstacle(_13, _B));

  std::list<nav::Point> slam_pntcloud;
  for (std::pair<nav::Point, nav::Point> &obs : slam_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          slam_pntcloud.push_back(pnt);
        }
      }
    }
  }
  {
    std::ofstream ofs("../data/slam_pntcloud.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << slam_pntcloud;
  }

  std::list<nav::Point> total_pntcloud;
  for (std::pair<nav::Point, nav::Point> &obs : nav_fix_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          total_pntcloud.push_back(pnt);
        }
      }
    }
  }
  for (std::pair<nav::Point, nav::Point> &obs : slam_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          total_pntcloud.push_back(pnt);
        }
      }
    }
  }
  {
    std::ofstream ofs("../data/total_pntcloud.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << total_pntcloud;
  }

  return 0;
}
