#include <iostream>
#include "geometry.h"
#include "tgaimage.h"

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
  if (t0.y == t1.y && t0.y == t2.y) return;

  // t2 is the highest in room
  if (t0.y > t1.y) std::swap(t0, t1);
  if (t0.y > t2.y) std::swap(t0, t2);
  if (t1.y > t2.y) std::swap(t1, t2);

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
}

// 2D -> 1D
void rasterize(Vec2i t0, Vec2i t1, TGAImage &image, TGAColor color, int y_buffer[]) {
  if (t0.x > t1.x) std::swap(t0, t1);

  float m = (t1.y - t0.y)/(float)(t1.x - t0.x);
  for (int x{t0.x}; x < t1.x; x++) {

    int y = t0.y + (x - t0.x)*m;
    if (y_buffer[x] > y) continue;

    y_buffer[x] = y;
    image.set(x, 0, color);
  }
}
