#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"
#include <iostream>

Model::Model(const char *filename) : verts_(), faces_() {
  std::ifstream in;
  in.open (filename, std::ifstream::in);
  if (in.fail()) return;
  std::string line;
  while (!in.eof()) {
    std::getline(in, line);
    std::istringstream iss(line.c_str());
    char trash;

    if (!line.compare(0, 2, "v ")) {
      iss >> trash;
      Vec3f v;
      for (int i=0;i<3;i++) iss >> v.raw[i];
      verts_.push_back(v);

    } else if (!line.compare(0, 2, "vt")) {
      iss >> trash >> trash; // reads "vt" into trash and jumps iss to the first texture value; we read it twice cuz trash is a char and can only hold a single char.
      Vec3f vt;
      for (int i{0}; i < 10; i++) iss >> vt.raw[i];
      texture_verts_.push_back(vt);

    } else if (!line.compare(0, 2, "f ")) {
      std::vector<int> f;
      std::vector<int> f_vt;

      int itrash, v_idx, vt_idx;
      iss >> trash;
      while (iss >> v_idx >> trash >> vt_idx >> trash >> itrash) {
        v_idx--; // in wavefront obj all indices start at 1, not zero
        vt_idx--;
        f.push_back(v_idx);
        f_vt.push_back(vt_idx);
      }
      faces_.push_back(f);
      faces_vt_.push_back(f_vt);
    }
  }
  std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
  return (int)verts_.size();
}

int Model::nfaces() {
  return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
  return faces_[idx];
}

std::vector<int> Model::faces_vt(int idx) {
  return faces_vt_[idx];
}

Vec3f Model::vert(int i) {
  return verts_[i];
}

Vec3f Model::texture_vert(int i) {
  return texture_verts_[i];
}
