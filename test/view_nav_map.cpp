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
#include "NavMap.h"

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

  nav::NavMap nav_map;
  {
    std::ifstream ifs("../data/nav_map.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> nav_map;
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
    gl::draw_xyplane(nav_map_xlen, nav_map_ylen);

    // Points
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::NavBox &box : nav_map.boxes()) {
      for (const nav::Point &pnt : box.fix_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::NavBox &box : nav_map.boxes()) {
      for (const nav::Point &pnt : box.slam_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();

    // Boxes
    glColor4f(0.2f, 0.2f, 0.2f, 0.1f);
    for (const nav::NavBox &box : nav_map.boxes()) {
      if (!box.is_free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), nav_map_xstep,
                     nav_map_ystep, nav_map_zstep);
    }

    // Links
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::NavBox &src : nav_map.boxes()) {
      for (nav::WtEdge edge : src.edges()) {
        if (edge.second < 0.42)
          continue;
        nav::NavBox dest = nav_map.boxes(edge.first);
        gl::draw_link(src.cnt().x(), src.cnt().y(), src.cnt().z(),
                      dest.cnt().x(), dest.cnt().y(), dest.cnt().z());
      }
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}