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
Vec3f light_dir = Vec3f(1, 1, -1).normalize();
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);
Model *model = NULL;

void triangle(Vec3f *pts, Vec3f *texture_pts, Vec3f *vertex_normals, float z_buffer[], TGAImage &image);
Vec3f barycentric(Vec3f *pts, Vec3f P);

// Model matrix expresses objects in world coordinates
// object (local) coords -> world
Matrix ModelMatrix = Matrix::identity(4);

/* Returns the View matrix which tranforms
 * the scene such that it appears as it
 * would appear if the camera was at `eye`
 *
 * Composed of the change of basis tranformation and translation for getting the eye/camera to the
 * origin. The order matters cuz we want the orientation transformation to happen relative to the
 * camera at the origin.
 *
 * View = M_inv * Tr;
 *
 * M^-1 = [x' y' z']( [x y z] - [cx cy cz] )^1
 * M^-1 = -[x' y' z'][cx cy cz]^-1
 * M_inv "undos" the rotation. So where the current basis vectors would land in a regular (i,j,k)
 * system.
 *
 **/
Matrix lookat(Vec3f eye, Vec3f center, Vec3f up) {
  Vec3f O_z = (eye - center).normalize();
  Vec3f O_x = (up ^ O_z).normalize(); // cross product
  Vec3f O_y = (O_z ^ O_x).normalize();
  
  Matrix M_inv = Matrix::identity(4);
  Matrix Tr = Matrix::identity(4);

  for (int i{0}; i < 3; i++) {
    M_inv[0][i] = O_x.raw[i];
    M_inv[1][i] = O_y.raw[i];
    M_inv[2][i] = O_z.raw[i];

    Tr[i][3] = -center.raw[i];
  }

  return M_inv * Tr;
}

// Matrix to map points in [-1,1] to image of [w, h]
Matrix viewport(int x, int y, int w, int h) {
  Matrix m = Matrix::identity(4);
  m[0][3] = x+w/2.f;
  m[1][3] = y+h/2.f;
  m[2][3] = depth/2.f;

  m[0][0] = w/2.f;
  m[1][1] = h/2.f;
  m[2][2] = depth/2.f;
  return m;
}

int main(int argc, char** argv) {

  std::cout << "Loading model." << std::endl;
  if (argc == 3) {
    model = new Model(argv[1], argv[2]);
  } else {
    model = new Model("./obj/african_head/african_head.obj", "./obj/african_head/african_head_diffuse.tga");
    //model = new Model("./obj/diablo3_pose/diablo3_pose.obj", "./obj/diablo3_pose/diablo3_pose_diffuse.tga");
  }

  float z_buffer[width*height];
  for (int i{0}; i < width*height; i++) {
    z_buffer[i] = -std::numeric_limits<float>::max();
  }
  std::cout << "Initialized z-buffer." << std::endl;

  Matrix Projection = Matrix::identity(4);
  Matrix Viewport = viewport(width/8, height/8, width*3/4, height*3/4);
  Matrix View = lookat(eye, center, Vec3f(0, 1, 0));

  Projection[3][2] = -1.f/(eye - center).norm();

  TGAImage render_image(width, height, TGAImage::RGB);

  std::cout << "Painting triangles --- ";
  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face_vert  = model->face_verts(i);
    std::vector<int> face_text  = model->face_texts(i);
    std::vector<int> face_norms = model->face_norms(i);

    Vec3f screen_coords[3];
    Vec3f world_coords[3];
    Vec3f vt[3];
    Vec3f vn[3];

    for (int j = 0; j < 3; j++) {
      Vec3f v  = model->vert(i, j);
      vt[j]    = model->texture_vert(i, j);
      vn[j]    = model->vert_norm(i, j);

      Vec3f temp        =  (Viewport * Projection * View * ModelMatrix * Matrix(v)).toVec3f();
      screen_coords[j]  = Vec3f(int(temp.x), int(temp.y), int(temp.z));

      world_coords[j]   = v;
    }

    //Vec3f n = ((world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0])).normalize();
    //float intensity = n*light_dir;

    triangle(screen_coords, vt, vn, z_buffer, render_image);
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
  render_image.flip_vertically();
  render_image.write_tga_file("debug-output.tga");
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
void triangle(Vec3f *pts, Vec3f *texture_pts, Vec3f *vertex_normals, float z_buffer[], TGAImage &image) {

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
  Vec3f n;
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

      /* Explanation for the P.z and texture interpolation below:
       * That's literally what barycentric coordinates mean!
       * Weighted avg of the cartesian coordiantes of the corners! (for each component)
       **/
      P.z = 0;
      Vec2f texture_coord(0., 0.);
      for (int i{0}; i < 3; i++) {
        P.z += pts[i].z * bary_coords.raw[i];

        texture_coord.x += texture_pts[i].x * bary_coords.raw[i];
        texture_coord.y += texture_pts[i].y * bary_coords.raw[i];

        n.x += vertex_normals[i].x * bary_coords.raw[i];
        n.y += vertex_normals[i].y * bary_coords.raw[i];
        n.z += vertex_normals[i].z * bary_coords.raw[i];
      }

      float intensity = n*light_dir;
      //if (intensity < 0) return;

      if (z_buffer[int(P.x + P.y * width)] >= P.z) continue;
      z_buffer[(int)(P.x + P.y * width)] = P.z;

      TGAColor color = model->diffuse(texture_coord);

      image.set(P.x, P.y, TGAColor(
        (color.r),
        (color.g),
        (color.b),
        255
      ));
    }

  }
}
