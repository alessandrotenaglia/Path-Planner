/**
 * @file generate_pntcloud.h
 * @brief Source file for the generation of the pointcloud
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

  std::list<std::pair<nav::Point, nav::Point>> fix_obstacles;

  nav::Point A11(0.0, 0.0, 0.0);
  nav::Point A12(1.0, 3.0, 1.9);
  auto A1 = std::make_pair(A11, A12);
  fix_obstacles.push_back(A1);

  nav::Point A21(1.0, 0.0, 0.0);
  nav::Point A22(2.0, 1.0, 1.9);
  auto A2 = std::make_pair(A21, A22);
  fix_obstacles.push_back(A2);

  nav::Point B11(1.0, 6.0, 0.0);
  nav::Point B12(3.3, 6.7, 1.2);
  auto B1 = std::make_pair(B11, B12);
  fix_obstacles.push_back(B1);

  nav::Point B21(1.0, 6.7, 0.0);
  nav::Point B22(1.6, 7.2, 1.2);
  auto B2 = std::make_pair(B21, B22);
  fix_obstacles.push_back(B2);

  nav::Point B31(2.7, 6.7, 0.0);
  nav::Point B32(3.3, 7.2, 1.2);
  auto B3 = std::make_pair(B31, B32);
  fix_obstacles.push_back(B3);

  nav::Point B41(1.0, 7.9, 0.0);
  nav::Point B42(1.6, 8.4, 1.2);
  auto B4 = std::make_pair(B41, B42);
  fix_obstacles.push_back(B4);

  nav::Point B51(2.7, 7.9, 0.0);
  nav::Point B52(3.3, 8.4, 1.2);
  auto B5 = std::make_pair(B51, B52);
  fix_obstacles.push_back(B5);

  nav::Point B61(1.0, 8.4, 0.0);
  nav::Point B62(3.3, 9.1, 1.2);
  auto B6 = std::make_pair(B61, B62);
  fix_obstacles.push_back(B6);

  nav::Point C11(4.8, 7.1, 0.0);
  nav::Point C12(5.5, 8.0, 1.6);
  auto C1 = std::make_pair(C11, C12);
  fix_obstacles.push_back(C1);

  nav::Point C21(5.5, 6.6, 0.0);
  nav::Point C22(6.0, 8.4, 1.6);
  auto C2 = std::make_pair(C21, C22);
  fix_obstacles.push_back(C2);

  nav::Point D11(8.0, 0.0, 0.0);
  nav::Point D12(8.8, 1.2, 3.0);
  auto D1 = std::make_pair(D11, D12);
  fix_obstacles.push_back(D1);

  nav::Point D21(8.0, 1.2, 2.5);
  nav::Point D22(8.8, 2.6, 3.0);
  auto D2 = std::make_pair(D21, D22);
  fix_obstacles.push_back(D2);

  nav::Point D31(8.0, 2.6, 0.0);
  nav::Point D32(8.8, 3.8, 3.0);
  auto D3 = std::make_pair(D31, D32);
  fix_obstacles.push_back(D3);

  nav::Point D41(8.8, 0.0, 2.5);
  nav::Point D42(10.0, 0.8, 3.0);
  auto D4 = std::make_pair(D41, D42);
  fix_obstacles.push_back(D4);

  nav::Point D51(8.8, 3.0, 2.5);
  nav::Point D52(10.0, 3.8, 3.0);
  auto D5 = std::make_pair(D51, D52);
  fix_obstacles.push_back(D5);

  nav::Point D61(10.0, 0.0, 0.0);
  nav::Point D62(10.7, 1.2, 3.0);
  auto D6 = std::make_pair(D61, D62);
  fix_obstacles.push_back(D6);

  nav::Point D71(10.0, 1.2, 2.5);
  nav::Point D72(10.7, 2.6, 3.0);
  auto D7 = std::make_pair(D71, D72);
  fix_obstacles.push_back(D7);

  nav::Point D81(10.0, 2.6, 0.0);
  nav::Point D82(10.7, 3.8, 3.0);
  auto D8 = std::make_pair(D81, D82);
  fix_obstacles.push_back(D8);

  nav::Point E11(8.0, 5.8, 0.0);
  nav::Point E12(10.5, 7.2, 2.0);
  auto E1 = std::make_pair(E11, E12);
  fix_obstacles.push_back(E1);

  nav::Point F11(11.0, 8.1, 0.0);
  nav::Point F12(15.7, 9.8, 0.7);
  auto F1 = std::make_pair(F11, F12);
  fix_obstacles.push_back(F1);

  nav::Point F21(12.7, 9.0, 0.7);
  nav::Point F22(15.7, 9.8, 2.0);
  auto F2 = std::make_pair(F21, F22);
  fix_obstacles.push_back(F2);

  nav::Point F31(14.8, 8.1, 0.7);
  nav::Point F32(15.7, 9.0, 2.0);
  auto F3 = std::make_pair(F31, F32);
  fix_obstacles.push_back(F3);

  nav::Point G11(13.5, 0.0, 0.0);
  nav::Point G12(15.9, 1.8, 0.7);
  auto G1 = std::make_pair(G11, G12);
  fix_obstacles.push_back(G1);

  nav::Point G21(13.5, 0.0, 0.7);
  nav::Point G22(14.6, 1.3, 1.9);
  auto G2 = std::make_pair(G21, G22);
  fix_obstacles.push_back(G2);

  nav::Point G31(15.9, 0.4, 0.0);
  nav::Point G32(17.0, 1.3, 1.4);
  auto G3 = std::make_pair(G31, G32);
  fix_obstacles.push_back(G3);

  nav::Point H11(16.0, 3.0, 0.0);
  nav::Point H12(16.7, 4.1, 0.9);
  auto H1 = std::make_pair(H11, H12);
  fix_obstacles.push_back(H1);

  nav::Point H21(16.0, 5.3, 0.0);
  nav::Point H22(17.1, 6.0, 0.9);
  auto H2 = std::make_pair(H21, H22);
  fix_obstacles.push_back(H2);

  nav::Point H31(17.9, 3.0, 0.0);
  nav::Point H32(19.0, 3.7, 0.9);
  auto H3 = std::make_pair(H31, H32);
  fix_obstacles.push_back(H3);

  nav::Point H41(18.3, 4.9, 0.0);
  nav::Point H42(19.0, 6.0, 0.9);
  auto H4 = std::make_pair(H41, H42);
  fix_obstacles.push_back(H4);

  nav::Point I11(17.8, 7.8, 0.0);
  nav::Point I12(20.0, 10.0, 0.8);
  auto I1 = std::make_pair(I11, I12);
  fix_obstacles.push_back(I1);

  nav::Point I21(17.8, 7.8, 0.8);
  nav::Point I22(19.0, 9.0, 2.0);
  auto I2 = std::make_pair(I21, I22);
  fix_obstacles.push_back(I2);

  std::list<nav::Point> fix_pntcloud;
  for (std::pair<nav::Point, nav::Point> &obs : fix_obstacles) {
    for (float x = obs.first.x(); x < obs.second.x(); x = x + 0.1f) {
      for (float y = obs.first.y(); y < obs.second.y(); y = y + 0.1f) {
        for (float z = obs.first.z(); z < obs.second.z(); z = z + 0.1f) {
          nav::Point pnt(x, y, z);
          fix_pntcloud.push_back(pnt);
        }
      }
    }
  }
  {
    std::ofstream ofs("../data/fix_pntcloud.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << fix_pntcloud;
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

  return 0;
}
