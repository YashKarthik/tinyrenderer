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

struct GouraudShader : public IShader {
    Vec3f varying_intensity;

    virtual Vec3f vertex(int iface, int nthvert) {
        Vec3f gl_Vertex = (model->vert(iface, nthvert));
        gl_Vertex = (Viewport * Projection * ModelView * gl_Vertex).toVec3f();

        varying_intensity.raw[nthvert] = std::max(0.f, model->vert_norm(iface, nthvert)*light_dir);
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        float intensity = varying_intensity*bar;
        color = TGAColor(255, 255, 255, 255)*intensity;
        return false;
    }
};


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

  projection(-1.f/(eye - center).norm());
  viewport(width/8, height/8, width*3/4, height*3/4);
  lookat(eye, center, Vec3f(0, 1, 0));
  TGAImage render_image(width, height, TGAImage::RGB);
  GouraudShader shader;

  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face_vert  = model->face_verts(i);
    std::vector<int> face_text  = model->face_texts(i);
    std::vector<int> face_norms = model->face_norms(i);

    Vec3f screen_coords[3];
    Vec3f vt[3];
    Vec3f vn[3];

    for (int j = 0; j < 3; j++) {
      vt[j]    = model->texture_vert(i, j);
      vn[j]    = model->vert_norm(i, j);

      Vec3f temp        =  shader.vertex(i, j);
      screen_coords[j]  = Vec3f(int(temp.x), int(temp.y), int(temp.z));
    }

    triangle(screen_coords, vt, vn, z_buffer, render_image);
  }
  std::cout << "Done." << std::endl;

  render_image.flip_vertically();
  render_image.write_tga_file("debug-output.tga");
  delete model;
  return 0;
}
