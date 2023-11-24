#include "tgaimage.h"
#include <fstream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(37, 164, 249, 1);
const TGAColor yellow = TGAColor(253, 190, 0, 255);
const TGAColor black = TGAColor(0, 0, 0, 255);

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color);

int main() {
  TGAImage image(100, 100, TGAImage::RGB);

  image.set(50, 10, white);
  image.set(50, 12, blue);

  line(10, 10, 30, 30, image, white);
  line(30, 30, 10, 10, image, red);

  line(10, 20, 80, 30, image, white);
  line(10, 10, 80, 30, image, blue);

  line(40, 5, 50, 90, image, yellow);

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

  for (int x{x0}; x < x1; x++) {
    int y = y0 + (x - x0)*(y1 - y0)/(float)(x1 - x0);

    if (steep) image.set(y, x, color);
    else image.set(x, y, color);
  }
}
