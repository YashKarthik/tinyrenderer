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
#include "gl.h"

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


int main() {

  model = new Model("./obj/african_head/african_head.obj", "./obj/african_head/african_head_diffuse.tga");
  //model = new Model("./obj/diablo3_pose/diablo3_pose.obj", "./obj/diablo3_pose/diablo3_pose_diffuse.tga");

  float z_buffer[width*height];
  for (int i{0}; i < width*height; i++) {
    z_buffer[i] = -std::numeric_limits<float>::max();
  }

  Matrix Projection = Matrix::identity(4);
  Matrix Viewport = viewport(width/8, height/8, width*3/4, height*3/4);
  Matrix ModelView = lookat(eye, center, Vec3f(0, 1, 0));

  Projection[3][2] = -1.f/(eye - center).norm();

  TGAImage render_image(width, height, TGAImage::RGB);

  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face_vert  = model->face_verts(i);
    std::vector<int> face_text  = model->face_texts(i);
    std::vector<int> face_norms = model->face_norms(i);

    Vec3f screen_coords[3];
    Vec3f world_coords[3];
    Vec3f vt[3];
    Vec3f vn[3];

    for (int j = 0; j < 3; j++) {
      Vec3f v  = model->vert(face_vert[j]);
      vt[j]    = model->texture_vert(face_text[j]);
      vn[j]    = model->vert_norm(0);

      Vec3f temp        =  (Viewport * Projection * ModelView * Matrix(v)).to_vector();
      screen_coords[j]  = Vec3f(int(temp.x), int(temp.y), int(temp.z));

      world_coords[j]   = v;
    }

    triangle(screen_coords, vt, vn, z_buffer, render_image);
  }

  render_image.flip_vertically();
  render_image.write_tga_file("output.tga");
  delete model;
  return 0;
}
