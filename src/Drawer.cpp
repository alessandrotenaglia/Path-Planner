/**
 * @file Drawer.cpp
 * @brief Source file for class Drawer
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/
#include "Drawer.h"

/*---------------------------------------------------------------------------*/
/*                             Methods Definition                            */
/*---------------------------------------------------------------------------*/
namespace gl {

void draw_axes() {
  // Save current opt
  glPushAttrib(GL_CURRENT_BIT);
  // X-Axis
  glColor3f(1.0f, 0.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(1.0f, 0.0f, 0.0f);
  glEnd();
  // Y-Axis
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glEnd();
  // Z-Axis
  glColor3f(0.0f, 0.0f, 1.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 1.0f);
  glEnd();
  // Restore previous opt
  glPopAttrib();
}

void draw_xyplane(float xlen, float ylen) {
  glBegin(GL_POLYGON);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(xlen, 0.0f, 0.0f);
  glVertex3f(xlen, ylen, 0.0f);
  glVertex3f(0.0f, ylen, 0.0f);
  glEnd();
}

void draw_yzplane(float ylen, float zlen) {
  glBegin(GL_POLYGON);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0, ylen, 0.0f);
  glVertex3f(0.0, ylen, zlen);
  glVertex3f(0.0f, 0.0, zlen);
  glEnd();
}

void draw_xzplane(float xlen, float zlen) {
  glBegin(GL_POLYGON);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(xlen, 0.0f, 0.0f);
  glVertex3f(xlen, 0.0f, zlen);
  glVertex3f(0.0f, 0.0f, zlen);
  glEnd();
}

void draw_box(float x, float y, float z, float xlen, float ylen, float zlen) {
  glPushMatrix();
  glTranslatef(x, y, z);
  glPushMatrix();
  glTranslatef(-xlen / 2, -ylen / 2, -zlen / 2);
  gl::draw_xyplane(xlen, ylen);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(-xlen / 2, -ylen / 2, -zlen / 2);
  gl::draw_yzplane(ylen, zlen);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(-xlen / 2, -ylen / 2, -zlen / 2);
  gl::draw_xzplane(xlen, zlen);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(-xlen / 2, -ylen / 2, zlen / 2);
  gl::draw_xyplane(xlen, ylen);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(xlen / 2, -ylen / 2, -zlen / 2);
  gl::draw_yzplane(ylen, zlen);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(-xlen / 2, ylen / 2, -zlen / 2);
  gl::draw_xzplane(xlen, zlen);
  glPopMatrix();
  glPopMatrix();
}

void draw_link(float xs, float ys, float zs, float xt, float yt, float zt) {
  glBegin(GL_LINES);
  glVertex3f(xs, ys, zs);
  glVertex3f(xt, yt, zt);
  glEnd();
}

void draw_circle(float radius) {
  glBegin(GL_POLYGON);
  for (int i = 0; i <= 20; i++) {
    double theta = 2 * M_PI * i / 20;
    double x = cos(theta) * radius;
    double y = sin(theta) * radius;
    glVertex2d(x, y);
  }
  glEnd();
}

void draw_cylinder(float x, float y, float z, float radius, float height) {
  glPushMatrix();
  glTranslatef(x, y, z);
  GLUquadricObj *quadratic = gluNewQuadric();
  glPushMatrix();
  glTranslatef(0.0, 0.0, -height);
  draw_circle(radius);
  gluCylinder(quadratic, radius, radius, 2 * height, 20, 1);
  glTranslatef(0.0, 0.0, 2 * height);
  draw_circle(radius);
  glPopMatrix();
  glPopMatrix();
}

} // namespace gl