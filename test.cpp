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

int main() {
  Model *model = new Model("./obj/african_head.obj");
  std::cout << std::endl << std::endl;

  std::vector<int> f_vt = model->faces_vt(2);
  Vec3f vt = model->texture_vert(f_vt[0]);
  std::cout << f_vt[0] << " " << vt << std::endl;
}
