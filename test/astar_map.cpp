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
std::list<nav::Point> get_pntcloud(nav::AStar astar, size_t curr_ind, float yaw,
                                   std::list<nav::Point> slam_pntcloud) {
  float right_dist = 1.0f;
  float left_dist = 1.0f;
  float front_dist = 3.0f;
  float back_dist = 1.0f;
  nav::Point curr_pnt = astar.map().boxes(curr_ind).cnt();
  nav::Point p1(curr_pnt.x() - back_dist, curr_pnt.y() - right_dist,
                curr_pnt.z());
  p1.rotate_xy(yaw);
  nav::Point p2(curr_pnt.x() + front_dist, curr_pnt.y() - right_dist,
                curr_pnt.z());
  p2.rotate_xy(yaw);
  nav::Point p3(curr_pnt.x() + front_dist, curr_pnt.y() + left_dist,
                curr_pnt.z());
  p3.rotate_xy(yaw);
  nav::Point p4(curr_pnt.x() - back_dist, curr_pnt.y() + left_dist,
                curr_pnt.z());
  p4.rotate_xy(yaw);
  std::vector<nav::Point> bounds = {p1, p2, p3, p4};
  std::list<nav::Point> pntcloud;
  for (nav::Point &pnt : slam_pntcloud) {
    if (pnt.is_inside(bounds) && abs(pnt.z() - curr_pnt.z()) < 1.0f)
      pntcloud.push_back(pnt);
  }
  return pntcloud;
}

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

  nav::Point trg_pnt(17.5, 4.5, 1.5);
  nav::Point str_pnt(2.5, 2.5, 1.5);

  std::list<const nav::Box *> path_from;
  std::list<const nav::Box *> *path_to;
  try {
    astar.set_str(str_pnt);
    astar.set_trg(trg_pnt);
    astar.compute_shortest_path();
    path_to = astar.path(astar.str());
  } catch (const char *err_msg) {
    std::cerr << err_msg << std::endl;
    exit(EXIT_FAILURE);
  }

  size_t curr_ind = astar.str();
  size_t next_ind = path_to->front()->ind();
  float yaw = astar.map().boxes(curr_ind).cnt().angle_xy(
      astar.map().boxes(next_ind).cnt());
  std::cout << astar.map().boxes(curr_ind).cnt() << " -> "
            << astar.map().boxes(next_ind).cnt() << " : " << yaw << std::endl;
  size_t cnt = 0;
  size_t fps = 1;

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
          std::list<nav::Point> pntcloud =
              get_pntcloud(astar, curr_ind, yaw, slam_pntcloud);
          astar.update(curr_ind, pntcloud);
          path_to = astar.path(curr_ind);
          path_from.push_back(&astar.map().boxes(curr_ind));
          //
          curr_ind = path_to->front()->ind();
          if (curr_ind != astar.trg()) {
            next_ind = (*std::next(path_to->begin(), 1))->ind();
            yaw = astar.map().boxes(curr_ind).cnt().angle_xy(
                astar.map().boxes(next_ind).cnt());
            //
            std::cout << astar.map().boxes(curr_ind).cnt() << " -> "
                      << astar.map().boxes(next_ind).cnt() << " : " << yaw
                      << std::endl;
          }
        } catch (const char *err_msg) {
          std::cerr << err_msg << std::endl;
          exit(EXIT_FAILURE);
        }
      }
    }

    cnt++;

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
    /*glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::Box &box : astar.map().boxes()) {
      if (!box.free()) //(!b.inside() || !b.free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), map_xstep,
                     map_ystep, map_zstep);
    }*/

    // Path
    if (curr_ind != astar.trg()) {
      glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
      nav::Box box = astar.map().boxes(curr_ind);
      gl::draw_cylinder(box.cnt().x(), box.cnt().y(), box.cnt().z(),
                        drone_radius, drone_height);
      glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
      for (const nav::Box *box : *path_to) {
        gl::draw_cylinder(box->cnt().x(), box->cnt().y(), box->cnt().z(),
                          drone_radius, drone_height);
      }
    } else {
      glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
      for (const nav::Box *box : path_from) {
        gl::draw_cylinder(box->cnt().x(), box->cnt().y(), box->cnt().z(),
                          drone_radius, drone_height);
      }
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}