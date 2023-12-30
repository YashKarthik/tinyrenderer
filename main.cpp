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

const int width = 800;
const int height = 800;
const int depth = 255;
Vec3f light_dir = Vec3f(1, 1, -1).normalize();
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);
Model *model = NULL;


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

  projection(-1.f/(eye - center).norm());
  viewport(width/8, height/8, width*3/4, height*3/4);
  lookat(eye, center, Vec3f(0, 1, 0));

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

      Vec3f temp        =  (Viewport * Projection * View * Matrix(v)).toVec3f();
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
