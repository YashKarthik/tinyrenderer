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

  /* Responsibilities:
  *   1. Reads (face, vertex) data from file.
  *   2. Calculates intensity at vertices.
  *   3. Transfroms scene's face into screen coordinates.
  **/
  virtual Vec3f vertex(int iface, int nthvert) {
    varying_intensity.raw[nthvert] = std::max(0.f, model->vert_norm(iface, nthvert)*light_dir);
    Matrix gl_Vertex = Matrix(model->vert(iface, nthvert));

    return (Viewport*Projection*ModelView*gl_Vertex).to_Vec3f();
  }

  /* Responsibilities:
   *  1. Interpolates intensity values at current pixel.
   *  2. Returns calculated color.
   * */
  virtual bool fragment(Vec3f bar, TGAColor &color) {
    float intensity = varying_intensity*bar;
    color = TGAColor(255*intensity, 255*intensity, 255*intensity, 255);
    return false;
  }
};


int main() {

  model = new Model("./obj/african_head/african_head.obj", "./obj/african_head/african_head_diffuse.tga");
  //model = new Model("./obj/diablo3_pose/diablo3_pose.obj", "./obj/diablo3_pose/diablo3_pose_diffuse.tga");

  float z_buffer[width*height];
  for (int i{0}; i < width*height; i++) {
    z_buffer[i] = -std::numeric_limits<float>::max();
  }

  projection(-1.f/(eye - center).norm());
  viewport(width/8, height/8, width*3/4, height*3/4);
  lookat(eye, center, Vec3f(0, 1, 0));

  TGAImage rendered_image(width, height, TGAImage::RGB);
  GouraudShader shader;

  for (int i = 0; i < model->nfaces(); i++) {
    Vec3f screen_coords[3];

    for (int j{0}; j < 3; j++) {
      screen_coords[j] = shader.vertex(i, j);
    }

    triangle(screen_coords, shader, rendered_image, z_buffer);
  }

  rendered_image.flip_vertically();
  rendered_image.write_tga_file("output.tga");
  delete model;
  return 0;
}
