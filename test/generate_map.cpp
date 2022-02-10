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
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                              Main Definition                             */
/*---------------------------------------------------------------------------*/
int main() {
  std::cout << "Il godo..." << std::endl;

  // Load config file
  cv::FileStorage fs;
  fs.open("../config/map_config.yaml", cv::FileStorage::READ);
  // Map config
  cv::FileNode map_cfg = fs["map"];
  float map_xlen = (float)map_cfg["xlen"];
  float map_ylen = (float)map_cfg["ylen"];
  float map_zlen = (float)map_cfg["zlen"];
  int map_nx = (int)map_cfg["nx"];
  int map_ny = (int)map_cfg["ny"];
  int map_nz = (int)map_cfg["nz"];
  float map_xstep = map_xlen / (float)map_nx;
  float map_ystep = map_ylen / (float)map_ny;
  float map_zstep = map_zlen / (float)map_nz;
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

  std::list<nav::Point> fix_pntcloud;
  {
    std::ifstream ifs("../data/fix_pntcloud.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> fix_pntcloud;
  }

  nav::Map pre_map;
  try {
    pre_map = nav::Map(map_xlen, map_ylen, map_zlen, map_nx, map_ny, map_nz,
                       drone_radius, drone_height, fix_pntcloud);
  } catch (const char *msg) {
    std::cerr << msg << std::endl;
    exit(EXIT_FAILURE);
  }
  {
    std::ofstream ofs("../data/map.dat");
    boost::archive::binary_oarchive oa(ofs);
    oa << pre_map;
  }

  return 0;
}