/**
 * @file Drawer.h
 * @brief Header file for class Drawer
 * @date 11 January 2022
 * @author Alessandro Tenaglia
 */

#ifndef DRAWER_H
#define DRAWER_H

/*---------------------------------------------------------------------------*/
/*                          Standard header includes                         */
/*---------------------------------------------------------------------------*/
#include <GL/glew.h>
#include <GL/glut.h>
#include <cmath>

/*---------------------------------------------------------------------------*/
/*                          Project header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                              Class Definition                             */
/*---------------------------------------------------------------------------*/
namespace gl {

void draw_axes();

void draw_xyplane(float xlen, float ylen);

void draw_yzplane(float ylen, float zlen);

void draw_xzplane(float xlen, float zlen);

void draw_box(float x, float y, float z, float xlen, float ylen, float zlen);

void draw_link(float xs, float ys, float zs, float xt, float yt, float zt);

void draw_circle(float radius);

void draw_cylinder(float x, float y, float z, float radius, float height);

} // namespace gl

#endif /* DRAWER_H */