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
#include "Explorer.h"

/*---------------------------------------------------------------------------*/
/*                              Main Definition                             */
/*---------------------------------------------------------------------------*/
int main() {
  std::cout << "Il godo..." << std::endl;

  // Load config file
  cv::FileStorage fs;
  fs.open("../config/map_config.yaml", cv::FileStorage::READ);
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

  nav::Explorer explorer;
  {
    std::ifstream ifs("../data/explorer.dat");
    boost::archive::binary_iarchive ia(ifs);
    ia >> explorer;
  }

  std::list<size_t> exp_path;
  nav::Point str_pnt(8.5, 8.5, 2.0);
  size_t curr_ind = explorer.pnt_to_ind(str_pnt);
  size_t curr_dir = 0;
  std::vector<size_t> path = explorer.generate_path(curr_ind, curr_dir);
  size_t trg_ind = path.back();
  std::vector<size_t> glb_path;
  glb_path.push_back(curr_ind);
  size_t cnt = 0, fps = 50;

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
    gl::draw_xyplane(exp_map_xlen, exp_map_ylen);

    // Boxes
    glColor4f(0.2f, 0.2f, 0.2f, 0.1f);
    for (const nav::ExpBox &box : explorer.boxes()) {
      if (!box.is_free())
        gl::draw_box(box.cnt().x(), box.cnt().y(), box.cnt().z(), exp_map_xstep,
                     exp_map_ystep, drone_height);
    }

    // Links
    glColor4f(0.2f, 0.2f, 0.2f, 0.2f);
    for (const nav::ExpBox &src : explorer.boxes()) {
      for (nav::WtEdge edge : src.edges()) {
        nav::ExpBox dest = explorer.boxes(std::get<0>(edge));
        gl::draw_link(src.cnt().x(), src.cnt().y(), src.cnt().z(),
                      dest.cnt().x(), dest.cnt().y(), dest.cnt().z());
      }
    }

    // Path
    glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
    gl::draw_box(explorer.boxes(curr_ind).cnt().x(),
                 explorer.boxes(curr_ind).cnt().y(),
                 explorer.boxes(curr_ind).cnt().z(), exp_map_xstep,
                 exp_map_ystep, drone_height);
    glColor4f(0.0f, 1.0f, 0.0f, 0.2f);
    for (size_t ind : glb_path) {
      gl::draw_box(explorer.boxes(ind).cnt().x(), explorer.boxes(ind).cnt().y(),
                   explorer.boxes(ind).cnt().z(), exp_map_xstep, exp_map_ystep,
                   drone_height);
    }

    if ((cnt % fps) == 0) {
      if (curr_ind != trg_ind) {
        curr_ind = path.front();
        path.erase(path.begin());
        glb_path.push_back(curr_ind);
      } else {
        curr_dir = explorer.choose_dir(curr_ind, curr_dir);
        path = explorer.generate_path(curr_ind, curr_dir);
        trg_ind = path.back();
      }
    }

    cnt++;

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
