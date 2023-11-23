#include "tgaimage.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor black = TGAColor(0, 0, 0, 255);

int main() {
  TGAImage image(100, 100, TGAImage::RGB);

  image.flip_vertically();
  image.set(19, 19, white);

  image.write_tga_file("output.tga");
  return 0;
}
