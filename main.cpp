#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <valarray>
#include <vector>
#include "model.h"
#include "geometry.h"
#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(37, 164, 249, 1);
const TGAColor yellow = TGAColor(253, 190, 0, 255);
const TGAColor black = TGAColor(0, 0, 0, 255);

const int width = 800;
const int height = 800;
const int depth = 255;

Model *model = NULL;

void triangle(Vec3f *pts, Vec3f *texture_pts, float z_buffer[], TGAImage &image, TGAImage &texture_image, float intensity);
void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color);
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);
Vec3f barycentric(Vec3f *pts, Vec3f P);
Vec2f interpolate_texture(Vec3f *texture_pts, Vec3f P, Vec3f P_bary);
void rasterize(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color, int y_buffer[]);

int main(int argc, char** argv) {
  TGAImage image(width, height, TGAImage::RGB);
  TGAImage texture_image = TGAImage();

  std::cout << "Loading model." << std::endl;
  if (argc == 2) {
    model = new Model(argv[1]);
  } else {
    model = new Model("./obj/african_head/african_head.obj");

    std::cout << "Loading textures" << std::endl;;
    if (!texture_image.read_tga_file("./obj/african_head/african_head_diffuse.tga")) {
      std::cout << "Failed to load textures." << std::endl;
      return 1;
    }
    texture_image.flip_vertically();
  }

  Vec3f light_dir(0, 0, -1);
  float z_buffer[width*height];
  for (int i{0}; i < width*height; i++) {
    z_buffer[i] = -std::numeric_limits<float>::max();
  }
  std::cout << "Initialized z-buffer." << std::endl;

  std::cout << "Painting triangles --- ";
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face_vert = model->face(i);
    std::vector<int> face_text = model->faces_vt(i);

    Vec3f screen_coords[3];
    Vec3f world_coords[3];
    Vec3f vt[3];

    for (int j = 0; j < 3; j++) {
      Vec3f v           = model->vert(face_vert[j]);
      screen_coords[j]  = Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), int((v.z+1.)*depth/2.));
      world_coords[j]   = v;
      vt[j] = model->texture_vert(face_text[j]);
    }

    Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
    n.normalize();
    float intensity = n*light_dir;

    triangle(screen_coords, vt, z_buffer, image, texture_image, intensity);
  }
  std::cout << "Done." << std::endl;

  std::cout << "Dumping z-buffer." << std::endl;
  TGAImage zb_image(width, height, TGAImage::GRAYSCALE);
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      zb_image.set(i, j, TGAColor(((z_buffer[i + j*width]))*255, 255));
    }
  }
  zb_image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
  zb_image.write_tga_file("zbuffer.tga");
  std::cout << "Done." << std::endl;

  std::cout << "Writing render to file." << std::endl;
  image.flip_vertically();
  image.write_tga_file("output.tga");
  delete model;
  return 0;
}

/* Returns the barycentric coordinates of a point>
 *
 * If any of the barycentric coordinates (the masses)
 * are negative, then the point P is outside the triangle.
 *
 * The cartesian coordinates of P are the weighted average of the corners of the triangle.
 * The "weights" are the barycentric coordinates.
 *
 * x = ux_1 + vx_2 + (1 - u - v)x_3
 * y = uy_1 + vy_2 + (1 - u - v)y_3
 * We replace the third weight with the expr since the sum of weights needs to be 1.
 *
 * Rearranging:
 * 0 = (x_3 - x) + u(x_1 - x_3) + v(x_2 - x_3)
 * 0 = (y_3 - y) + u(y_1 - y_3) + v(y_2 - y_3)
 *
 * Rewriting as matrix-vector product:
 * [1 u v][x_3 - x  x_1 - x_3   x_2 - x_3]^T = 0
 * [1 u v][y_3 - y  y_1 - y_3   y_2 - y_3]^T = 0
 *
 * So we need a vector that is perpendicular to both
 * [x_3 - x  x_1 - x_3   x_2 - x_3]^T and [y_3 - y  y_1 - y_3   y_2 - y_3]^T
 * => [1 u v] =(approx) [x_3 - x  x_1 - x_3   x_2 - x_3]^T \cross [y_3 - y  y_1 - y_3   y_2 - y_3]^T
 **/
Vec3f barycentric(Vec3f *pts, Vec3f P) {

  Vec3f cross = Vec3f(
    pts[2].x - pts[0].x,
    pts[1].x - pts[0].x,
    pts[0].x - P.x
  ) ^ Vec3f(
      pts[2].y - pts[0].y,
      pts[1].y - pts[0].y,
      pts[0].y - P.y
  );

  // Points are colinear => zero area => return neg to signify point is outside
  if (std::abs(cross.z) <= 1e-2) return Vec3f(-1, 1, 1);

  // div by z to scale it down so that sum is 1
  // The third coordinate is = 1 - (u + v)
  return Vec3f(
    1.f - (cross.x + cross.y)/cross.z,
    cross.y/cross.z,
    cross.x/cross.z
  );
}


/* 3D -> 2D rasterization
 * Using barycentric coordinates and z-buffers
 *
 * Accesing z_buffer:
 *  int idx = x + y*width;
 *  int x = idx % width;
 *  int y = idx / width;
 * */
void triangle(Vec3f *pts, Vec3f *texture_pts, float z_buffer[], TGAImage &image, TGAImage &texture_image, float intensity) {
  if (intensity < 0) return;

  Vec2f bounding_box_min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  Vec2f bounding_box_max(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
  Vec2f clamp(image.get_width()-1, image.get_height()-1);

  // go over each point and store the lowest, highest possible x-cord and y-cord
  for (int i{0}; i < 3; i++) {
    // comparing the current cordinate with the smallest cordinate encountered so far.
    bounding_box_min.x = std::max(0.f, std::min(pts[i].x, bounding_box_min.x));
    bounding_box_min.y = std::max(0.f, std::min(pts[i].y, bounding_box_min.y));
    // wrapped in additional std::max to ensure vals are > 0

    // comparing the current point with the largest coordinate encountered so far
    bounding_box_max.x = std::min(clamp.x, std::max(pts[i].x, bounding_box_max.x));
    bounding_box_max.y = std::min(clamp.y, std::max(pts[i].y, bounding_box_max.y));
    // wrapped in additional std::min to ensure vals are < image size
  }

  Vec3f P;
  /* For each vector in the bounding box
   * Check if it's in (2d proj of) the triangle too.
   * Check if it's the forward-most point in the z-buffer.
   * If both are true, then color it.
   * */
  for (P.x = bounding_box_min.x; P.x <= bounding_box_max.x; P.x++) {
    for (P.y = bounding_box_min.y; P.y <= bounding_box_max.y; P.y++) {
      Vec3f bary_coords = barycentric(pts, P);
      // if any of the masses are negative; the point is outside the triangle
      if (bary_coords.x < 0 || bary_coords.y < 0 || bary_coords.z < 0) continue;

      /* Why is P.z = pts[i].z * bary_coords.x/y/z?
       * Manipulating the y = mx + c representation in the rasterize function
       * We get int y = p0.y*(1.-t) + p1.y*t; where (t, 1-t) are the barycentric coordinates of the point.
       * So the coordinate is basially the sum of the points' z coord * bary coords.
       * */
      P.z = 0;
      Vec2f texture_coord(0., 0.);
      for (int i{0}; i < 3; i++) {
        P.z += pts[i].z * bary_coords.raw[i];
        texture_coord.x += texture_pts[i].x * bary_coords.raw[i];
        texture_coord.y += texture_pts[i].y * bary_coords.raw[i];
      }

      if (z_buffer[int(P.x + P.y * width)] >= P.z) continue;
      z_buffer[(int)(P.x + P.y * width)] = P.z;

      TGAColor color = texture_image.get(
        (int)(texture_coord.x * texture_image.get_width()),
        (int)(texture_coord.y * texture_image.get_height())
      );

      image.set(P.x, P.y, TGAColor(
        intensity*(color.r),
        intensity*(color.g),
        intensity*(color.b),
        255
      ));
    }

  }
}
