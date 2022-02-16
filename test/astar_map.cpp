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
  nav::AStar astar(map);
  std::list<nav::Point> slam_pntcloud;
  {
    std::ifstream ifs("../data/slam_pntcloud.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> slam_pntcloud;
  }
  std::list<nav::Point> total_pntcloud;
  {
    std::ifstream ifs("../data/total_pntcloud.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> total_pntcloud;
  }

  nav::Point str_pnt(17.5, 4.5, 1.5);
  nav::Point trg_pnt(2.5, 2.5, 1.5);

  std::list<size_t> path_from;
  std::list<size_t> path_to;
  try {
    astar.set_str(str_pnt);
    astar.set_trg(trg_pnt);
    astar.compute_shortest_path();
    path_to = astar.path();
  } catch (const char *err_msg) {
    std::cerr << err_msg << std::endl;
    exit(EXIT_FAILURE);
  }

  size_t curr_ind = astar.str(), next_ind;
  nav::Point curr_pnt, next_pnt;
  double yaw;
  size_t cnt = 0;
  size_t fps = 50;
  nav::Point p1, p2, p3, p4, p5, p6;
  std::vector<nav::Point> bounds;

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

    if ((cnt % fps) == 0) {
      if (curr_ind != astar.trg()) {
        try {
          curr_ind = astar.str();
          curr_pnt = astar.map().boxes(curr_ind).cnt();
          next_ind = astar.path().front();
          next_pnt = astar.map().boxes(next_ind).cnt();
          if ((curr_pnt.x() != next_pnt.x()) || (curr_pnt.y() != next_pnt.y()))
            yaw = curr_pnt.angle_xy(next_pnt);
          std::cout << curr_pnt << " -> " << next_pnt << " : " << yaw
                    << std::endl;
          //
          p1.set(curr_pnt.x() - 1.0f, curr_pnt.y() - 1.0f, curr_pnt.z());
          p1.rotate_xy(curr_pnt, yaw);
          p2.set(curr_pnt.x() + 3.0f, curr_pnt.y() - 1.0f, curr_pnt.z());
          p2.rotate_xy(curr_pnt, yaw);
          p3.set(curr_pnt.x() + 3.0f, curr_pnt.y() + 1.0f, curr_pnt.z());
          p3.rotate_xy(curr_pnt, yaw);
          p4.set(curr_pnt.x() - 1.0f, curr_pnt.y() + 1.0f, curr_pnt.z());
          p4.rotate_xy(curr_pnt, yaw);
          bounds = {p1, p2, p3, p4};
          std::list<nav::Point> pntcloud;
          for (nav::Point &pnt : total_pntcloud) {
            if (pnt.is_inside_xy(bounds) && abs(curr_pnt.z() - pnt.z()) <= 1.0f)
              pntcloud.push_back(pnt);
          }
          //
          astar.update(pntcloud);
          path_to = astar.path();
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
    /*glColor4f(0.2f, 0.2f, 0.2f, 0.05f);
    for (const nav::Box &box : astar.map().boxes()) {
      if (!box.is_free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), map_xstep,
                     map_ystep, map_zstep);
    }*/

    //
    glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
    gl::draw_polyhedron(p1, p2, p3, p4, 1.0f);

    // Path
    if (curr_ind != astar.trg()) {
      glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
      nav::Box box = astar.map().boxes(curr_ind);
      gl::draw_cylinder(box.cnt().x(), box.cnt().y(), box.cnt().z(),
                        drone_radius, drone_height);
      glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
      for (size_t ind : path_to) {
        nav::Box box = astar.map().boxes(ind);
        gl::draw_cylinder(box.cnt().x(), box.cnt().y(), box.cnt().z(),
                          drone_radius, drone_height);
      }
    } else {
      glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
      for (size_t ind : path_from) {
        nav::Box box = astar.map().boxes(ind);
        gl::draw_cylinder(box.cnt().x(), box.cnt().y(), box.cnt().z(),
                          drone_radius, drone_height);
      }
    }

    if ((cnt % fps) == (fps / 2)) {
      if (curr_ind != astar.trg()) {
        try {
          path_from.push_back(curr_ind);
          curr_ind = astar.next();
        } catch (const char *err_msg) {
          std::cerr << err_msg << std::endl;
          exit(EXIT_FAILURE);
        }
      }
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
    cnt++;
  }

  return 0;
}
