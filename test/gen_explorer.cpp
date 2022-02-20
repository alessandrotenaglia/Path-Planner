/**
 * @file generate_pntcloud.h
 * @brief Source file for the generation of the map
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "ExpMap.h"

/*---------------------------------------------------------------------------*/
/*                              Main Definition                             */
/*---------------------------------------------------------------------------*/
int main() {
  std::cout << "Il godo..." << std::endl;

  // Load config file
  cv::FileStorage fs;
  fs.open("../config/map_config.yaml", cv::FileStorage::READ);
  // Nav Map config
  cv::FileNode nav_map_cfg = fs["nav_map"];
  float nav_map_xlen = (float)nav_map_cfg["xlen"];
  float nav_map_ylen = (float)nav_map_cfg["ylen"];
  float nav_map_zlen = (float)nav_map_cfg["zlen"];
  int nav_map_nx = (int)nav_map_cfg["nx"];
  int nav_map_ny = (int)nav_map_cfg["ny"];
  int nav_map_nz = (int)nav_map_cfg["nz"];
  float nav_map_xstep = nav_map_xlen / (float)nav_map_nx;
  float nav_map_ystep = nav_map_ylen / (float)nav_map_ny;
  float nav_map_zstep = nav_map_zlen / (float)nav_map_nz;
  // Exp Map config
  cv::FileNode exp_map_cfg = fs["exp_map"];
  float exp_map_xlen = (float)exp_map_cfg["xlen"];
  float exp_map_ylen = (float)exp_map_cfg["ylen"];
  float exp_map_zlen = (float)exp_map_cfg["zlen"];
  int exp_map_nx = (int)exp_map_cfg["nx"];
  int exp_map_ny = (int)exp_map_cfg["ny"];
  int exp_map_nz = (int)exp_map_cfg["nz"];
  float exp_map_xstep = exp_map_xlen / (float)exp_map_nx;
  float exp_map_ystep = exp_map_ylen / (float)exp_map_ny;
  float exp_map_zstep = exp_map_zlen / (float)exp_map_nz;
  // Drone config
  cv::FileNode drone_cfg = fs["drone"];
  float drone_radius = (float)drone_cfg["radius"];
  float drone_height = (float)drone_cfg["height"];
  // Window config
  cv::FileNode window_cfg = fs["window"];
  std::string window_name = (std::string)window_cfg["name"];
  int window_width = (int)window_cfg["width"];
  int window_height = (int)window_cfg["height"];
  double window_fview = (double)window_cfg["fview"];
  double window_xview = (double)window_cfg["xview"];
  double window_yview = (double)window_cfg["yview"];
  double window_zview = (double)window_cfg["zview"];
  double window_xstart = (double)window_cfg["xstart"];
  double window_ystart = (double)window_cfg["ystart"];

  std::list<nav::Point> exp_fix_pntcloud;
  {
    std::ifstream ifs("../data/exp_fix_pntcloud.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> exp_fix_pntcloud;
  }

  nav::ExpMap exp_map;
  try {
    exp_map = nav::ExpMap(exp_map_xlen, exp_map_ylen, exp_map_nx, exp_map_ny,
                          drone_radius, exp_fix_pntcloud);
  } catch (const char *msg) {
    std::cerr << msg << std::endl;
    exit(EXIT_FAILURE);
  }
  {
    std::ofstream ofs("../data/exp_map.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << exp_map;
  }

  return 0;
}
