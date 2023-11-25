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

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);

int main(int argc, char** argv) {
  if (argc == 2) {
    model = new Model(argv[1]);
  } else {
    model = new Model("./obj/african_head.obj");
  }

  TGAImage image(width, height, TGAImage::RGB);
  for (int i{0}; i < model->nfaces(); i++) {

    std::vector<int> face = model->face(i);
    for (int j{0}; j < 3; j++) {

      Vec3f v0 = model->vert(face[j]);
      Vec3f v1 = model->vert(face[(j + 1) % 3]);

      int x0 = (v0.x + 1.0)*width/2.0;
      int y0 = (v0.y + 1.0)*width/2.0;
      int x1 = (v1.x + 1.0)*width/2.0;
      int y1 = (v1.y + 1.0)*width/2.0;

      line(x0, y0, x1, y1, image, white);
    }
  }

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
