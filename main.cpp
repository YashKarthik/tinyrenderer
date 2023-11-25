#include <iostream>
#include <fstream>
#include <cmath>
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

Model *model = NULL;

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color);
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);

int main(int argc, char** argv) {
  // if (argc == 2) {
  //   model = new Model(argv[1]);
  // } else {
  //   model = new Model("./obj/african_head.obj");
  // }

  TGAImage image(width, height, TGAImage::RGB);
  // for (int i{0}; i < model->nfaces(); i++) {

  //   std::vector<int> face = model->face(i);
  //   for (int j{0}; j < 3; j++) {

  //     Vec3f v0 = model->vert(face[j]);
  //     Vec3f v1 = model->vert(face[(j + 1) % 3]);

  //     int x0 = (v0.x + 1.0)*width/2.0;
  //     int y0 = (v0.y + 1.0)*width/2.0;
  //     int x1 = (v1.x + 1.0)*width/2.0;
  //     int y1 = (v1.y + 1.0)*width/2.0;

  //     line(x0, y0, x1, y1, image, white);
  //   }
  // }

  Vec2i t0[3] = {Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80)}; 
  Vec2i t1[3] = {Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180)}; 
  Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)}; 

  triangle(t0[0], t0[1], t0[2], image, red); 
  triangle(t1[0], t1[1], t1[2], image, blue); 
  triangle(t2[0], t2[1], t2[2], image, green);

  image.flip_vertically();
  image.write_tga_file("output.tga");
  return 0;
}

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
  bool steep = false;
  if (std::abs(y1 - y0) > std::abs(x1 - x0)) {
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }

  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  float m = (y1 - y0)/(float)(x1 - x0);
  for (int x{x0}; x < x1; x++) {
    int y = y0 + (x - x0)*m; // y = y0 + m*Dx

    if (steep) image.set(y, x, color);
    else image.set(x, y, color);
  }
}

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) { 

  // t2 is the highest in room
  if (t0.y > t1.y) {
    std::swap(t0, t1);
  }
  if (t0.y > t2.y) {
    std::swap(t0, t2);
    std::swap(t1, t2);
  }
  if (t1.y > t2.y) {
    std::swap(t1, t2);
  }

  /* Split the triangle into two sections based on the middle vertex.
   * We need to calculate the line eq for three lines now:
   * t0 -> t2; t0 -> t1; t1 -> t2
   *
   * Calculate as function of y cuz iterating over y is nice here: x = f(y)
   * For each y in [t1.y, t2.y], calculate x_left, x_right using f_2
   * For y in [t0.y, t1.y] use f_1
   * For y in [t0.y, t2.y] use f_2
   *
   * x_out = (y_in - y0)/m + x0
   **/
  float m0 = (t2.y - t0.y) / (float)(t2.x - t0.x);
  float m1 = (t1.y - t0.y) / (float)(t1.x - t0.x);
  float m2 = (t2.y - t1.y) / (float)(t2.x - t1.x);

  for (int y{t1.y}; y <= t2.y; y++) {
    int x0 = (y - t0.y)/m0 + t0.x;
    int x2 = (y - t1.y)/m2 + t1.x;

    line(x0, y, x2, y, image, color);
  }
  for (int y{t0.y}; y <= t1.y; y++) {
    int x0 = (y - t0.y)/m0 + t0.x;
    int x1 = (y - t1.y)/m1 + t1.x;

    line(x0, y, x1, y, image, color);
  }

  line(t0.x, t0.y, t1.x, t1.y, image, white);
  line(t0.x, t0.y, t2.x, t2.y, image, white);
  line(t2.x, t2.y, t1.x, t1.y, image, white);
}
