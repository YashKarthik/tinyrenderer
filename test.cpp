#include <iostream>
#include "model.h"
#include "geometry.h"
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

Vec3f interpolate_texture(Vec3f *texture_pts, Vec3f P_bary) {
  Vec3f interpolated_texture_coords;

  for (int i{0}; i < 3; i++) {
    interpolated_texture_coords.raw[i] = texture_pts->raw[i]*P_bary.raw[i];
  }

  return interpolated_texture_coords;
}

int main() {
  Model *model = new Model("./obj/african_head/african_head.obj");
  std::cout << std::endl << std::endl;

  std::vector<int> f_vt = model->faces_vt(2);
  Vec3f vt = model->texture_vert(f_vt[0]);

  Vec3f vts[] = {
    model->texture_vert(f_vt[0]),
    model->texture_vert(f_vt[1]),
    model->texture_vert(f_vt[2]),
  };

  Vec3f t = interpolate_texture(vts, Vec3f(1,1,1));

  std::cout << t << std::endl;
}
