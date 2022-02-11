/**
 * @file aster_map.h
 * @brief Source file for A*
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
#include "AStar.h"
#include "Drawer.h"

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

  nav::Map map;
  {
    std::ifstream ifs("../data/map.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> map;
  }

  std::list<nav::Point> slam_pntcloud;
  {
    std::ifstream ifs("../data/slam_pntcloud.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> slam_pntcloud;
  }

  nav::Point p_str(2.5, 2.5, 1.5);
  nav::Point p_trg(14.5, 6.0, 1.5);

  nav::AStar astar(map);
  try {
    astar.set_str(p_str);
    astar.set_trg(p_trg);
  } catch (const char *err_msg) {
    std::cerr << err_msg << std::endl;
    exit(EXIT_FAILURE);
  }

  size_t curr_ind = astar.str()->ind();
  std::list<const nav::Box *> path_from;
  path_from.push_back(&astar.map().boxes(curr_ind));
  std::list<nav::Box *> *path_to;

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

  size_t cnt = 0;
  size_t fps = 100;

  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 0.2f);

    // Compute shortest path
    if ((cnt % fps) == 0) {
      if (curr_ind != astar.trg()->ind()) {
        try {
          astar.compute_shortest_path();
          path_to = astar.path(curr_ind);
        } catch (const char *err_msg) {
          std::cerr << err_msg << std::endl;
          exit(EXIT_FAILURE);
        }
      }
    }

    // Origin
    gl::draw_axes();

    // Floor
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    gl::draw_xyplane(map_xlen, map_ylen);

    // Fixed points
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::Box &b : map.boxes()) {
      for (const nav::Point &pnt : b.fix_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();

    // SLAM points
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    for (const nav::Box &b : astar.map().boxes()) {
      for (const nav::Point &pnt : b.slam_pnts()) {
        glVertex3f(pnt.x(), pnt.y(), pnt.z());
      }
    }
    glEnd();

    // Boxes
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::Box &box : astar.map().boxes()) {
      if (!box.free()) //(!b.inside() || !b.free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), map_xstep,
                     map_ystep, map_zstep);
    }

    // Path
    glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
    for (const nav::Box *box : path_from) {
      gl::draw_box(box->cnt().x(), box->cnt().y(), box->cnt().z(), map_xstep,
                   map_ystep, map_zstep);
    }
    glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
    for (nav::Box *box : *path_to) {
      gl::draw_box(box->cnt().x(), box->cnt().y(), box->cnt().z(), map_xstep,
                   map_ystep, map_zstep);
    }

    if ((cnt % fps) == 0) {
      if (curr_ind != astar.trg()->ind()) {

        curr_ind = path_to->front()->ind();
        nav::Point curr_pnt = astar.map().boxes(curr_ind).cnt();
        astar.set_str(curr_pnt);
        path_from.push_back(&astar.map().boxes(curr_ind));

        nav::Point p1(curr_pnt.x() - 2, curr_pnt.y() - 2, curr_pnt.z() - 2);
        nav::Point p2(curr_pnt.x() + 2, curr_pnt.y() + 2, curr_pnt.z() + 2);
        std::list<nav::Point> pntcloud;
        for (const nav::Point &pnt : slam_pntcloud) {
          if ((p1.x() <= pnt.x() && pnt.x() <= p2.x()) &&
              (p1.y() <= pnt.y() && pnt.y() <= p2.y()) &&
              (p1.z() <= pnt.z() && pnt.z() <= p2.z()))
            pntcloud.push_back(pnt);
        }
        astar.update_map(pntcloud);
      }
    }

    cnt++;

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}