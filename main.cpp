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
Vec3f light_dir = Vec3f(1,1,1).normalize();
Vec3f eye(1, 1, 3);
Vec3f center(0, 0, 0);
Model *model = NULL;

struct Shader : public IShader {
  Vec3f varying_textures[3];

  // uniform is used to signify that these variables are constant for shader instance
  Matrix uniform_M;           // Projection * ModelView;
  Matrix uniform_MIT;         // (Projection * ModelView).invert_transpose();

  virtual Vec3f vertex(int iface, int nthvert) {
    varying_textures[nthvert] = model->texture_vert(iface, nthvert);

    Vec3f gl_Vertex = (model->vert(iface, nthvert));
    gl_Vertex = (Viewport * Projection * ModelView * gl_Vertex).to_Vec3f();
    return gl_Vertex;
  }

  virtual bool fragment(Vec3f bar, TGAColor &color) {

    Vec2f uv(0.f, 0.f);
    for (int i{0}; i < 3; i++) {
      uv.x += varying_textures[i].x * bar.raw[i];
      uv.y += varying_textures[i].y * bar.raw[i];
    }

    Vec3f n = (uniform_MIT * Matrix(model->normal(uv))).to_Vec3f().normalize();
    Vec3f l = (uniform_M * Matrix(light_dir)).to_Vec3f().normalize();
    float intensity = std::max(0.f, n*l);

    color = model->diffuse(uv)*intensity;
    return false;
  }
};


int main() {
  model = new Model(
    "./obj/african_head/african_head.obj",
    "./obj/african_head/african_head_diffuse.tga",
    "./obj/african_head/african_head_nm_tangent.tga"
  );
  //model = new Model("./obj/diablo3_pose/diablo3_pose.obj", "./obj/diablo3_pose/diablo3_pose_diffuse.tga");

  float z_buffer[width*height];
  for (int i{0}; i < width*height; i++) {
    z_buffer[i] = -std::numeric_limits<float>::max();
  }

  projection(-1.f/(eye - center).norm());
  viewport(width/8, height/8, width*3/4, height*3/4);
  lookat(eye, center, Vec3f(0, 1, 0));

  TGAImage render_image(width, height, TGAImage::RGB);
  Shader shader;

  shader.uniform_M = Projection * ModelView;
  shader.uniform_MIT = (Projection * ModelView).inverse().transpose();

  for (int i = 0; i < model->nfaces(); i++) {
    Vec3f screen_coords[3];
    Vec3f vt[3];
    Vec3f vn[3];

    for (int j = 0; j < 3; j++) {
      vt[j]    = model->texture_vert(i, j);
      vn[j]    = model->vert_norm(i, j);

      Vec3f temp        =  shader.vertex(i, j);
      screen_coords[j]  = Vec3f(int(temp.x), int(temp.y), int(temp.z));
    }

    //triangle(screen_coords, vt, vn, z_buffer, render_image);
    triangle(screen_coords, shader, render_image, z_buffer);
  }

  render_image.flip_vertically();
  render_image.write_tga_file("output.tga");
  delete model;
  return 0;
}
