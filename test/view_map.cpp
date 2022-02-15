/**
 * @file generate_pntcloud.h
 * @brief Source file for the view of the map
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Drawer.h"
#include "Map.h"

/*---------------------------------------------------------------------------*/
/*                              Main Definition                             */
/*---------------------------------------------------------------------------*/
int main() {
  std::cout << "Il godo..." << std::endl;

  float yaw = 0.0;
  nav::Point p1(0.0f, 0.0f, 0.0f);
  p1.rotate_xy(yaw);
  nav::Point p2(1.0f, 0.0f, 0.0f);
  p2.rotate_xy(yaw);
  nav::Point p3(1.0f, 1.0f, 0.0f);
  p3.rotate_xy(yaw);
  nav::Point p4(0.0f, 1.0f, 0.0f);
  p4.rotate_xy(yaw);
  std::vector<nav::Point> bounds = {p1, p2, p3, p4};
  nav::Point pnt(0.5, 0.5, 0.0);
  std::cout << pnt << std::endl;
  std::cout << pnt.is_inside(bounds) << std::endl;

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

  nav::Map map;
  {
    std::ifstream ifs("../data/map.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> map;
  }

  pangolin::CreateWindowAndBind(window_name, window_width, window_height);
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // Enable blending
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(window_width, window_height, window_fview,
                                 window_fview, window_xstart, window_ystart,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(window_xview, window_yview, window_zview, 0, 0,
                                0, pangolin::AxisZ));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, (float)-window_width / window_height)
          .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 0.2f);

    // Origin
    gl::draw_axes();

    // Floor
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    gl::draw_xyplane(map_xlen, map_ylen);

    // Points
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::Box &box : map.boxes()) {
      for (const nav::Point &pnt : box.fix_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::Box &box : map.boxes()) {
      for (const nav::Point &pnt : box.slam_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();

    /*//
    glColor4f(1.0f, 0.0f, 0.0f, 0.2f);
    nav::Point pnt(0.5, 2.5, 1.5);
    for (size_t neigh : map.boxes(map.pnt_to_ind(pnt)).neighs()) {
      gl::draw_box(map.boxes(neigh).cnt().x(), map.boxes(neigh).cnt().y(),
                   map.boxes(neigh).cnt().z(), map_xstep, map_ystep, map_zstep);
    }

    //
    glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
    nav::Point link_pnt(0.5, 4.5, 1.5);
    for (nav::WtEdge edge : map.boxes(map.pnt_to_ind(link_pnt)).edges()) {
      gl::draw_box(
          map.boxes(edge.first).cnt().x(), map.boxes(edge.first).cnt().y(),
          map.boxes(edge.first).cnt().z(), map_xstep, map_ystep, map_zstep);
    }*/

    // Boxes
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::Box &box : map.boxes()) {
      if (!box.free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), map_xstep,
                     map_ystep, map_zstep);
    }

    /*// Links
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::Box &src : map.boxes()) {
      for (nav::WtEdge edge : src.edges()) {
        if (edge.second < 0.42)
          continue;
        nav::Box dest = map.boxes(edge.first);
        gl::draw_link(src.cnt().x(), src.cnt().y(), src.cnt().z(),
                      dest.cnt().x(), dest.cnt().y(), dest.cnt().z());
      }
    }*/

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}