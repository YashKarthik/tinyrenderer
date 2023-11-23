#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(37, 164, 249, 1);
const TGAColor yellow = TGAColor(253, 190, 0, 255);
const TGAColor black = TGAColor(0, 0, 0, 255);

int main() {
  TGAImage image(100, 100, TGAImage::RGB);

  image.flip_vertically();

  for (std::size_t i{0}; i < 10; i++) {
    for (std::size_t j{0}; j < 10; j++) {
      image.set(10+j, 10+i, red);
    }
  }
  for (std::size_t i{0}; i < 10; i++) {
    for (std::size_t j{0}; j < 10; j++) {
      image.set(21+j, 10+i, green);
    }
  }
  for (std::size_t i{0}; i < 10; i++) {
    for (std::size_t j{0}; j < 10; j++) {
      image.set(10+j, 21+i, blue);
    }
  }
  for (std::size_t i{0}; i < 10; i++) {
    for (std::size_t j{0}; j < 10; j++) {
      image.set(21+j, 10+i, green);
    }
  }
  for (std::size_t i{0}; i < 10; i++) {
    for (std::size_t j{0}; j < 10; j++) {
      image.set(21+j, 21+i, yellow);
    }
  }

  image.write_tga_file("output.tga");
  return 0;
}
