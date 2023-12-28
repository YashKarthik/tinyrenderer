#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"
#include <iostream>

Model::Model(const char *obj_file, const char *texts_file) : verts(), faces_verts() {
  std::ifstream in;
  in.open (obj_file, std::ifstream::in);
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
      verts.push_back(v);

    } else if (!line.compare(0, 2, "vt")) {
      iss >> trash >> trash; // reads "vt" into trash and jumps iss to the first texture value; we read it twice cuz trash is a char and can only hold a single char.
      Vec3f vt;
      for (int i{0}; i < 10; i++) iss >> vt.raw[i];
      textures.push_back(vt);

    } else if (!line.compare(0, 2, "f ")) {
      std::vector<int> f_verts;
      std::vector<int> f_texts;
      std::vector<int> f_norms;

      int v_idx, vt_idx, n_idx;
      iss >> trash;
      while (iss >> v_idx >> trash >> vt_idx >> trash >> n_idx) {
        v_idx--; // in wavefront obj all indices start at 1, not zero
        vt_idx--;
        n_idx--;
        f_verts.push_back(v_idx);
        f_texts.push_back(vt_idx);
        f_norms.push_back(n_idx);
      }
      faces_verts.push_back(f_verts);
      faces_texts.push_back(f_texts);
      faces_norms.push_back(f_norms);
    }
  }
  load_textures(texts_file, diffuse_map);
  std::cerr << "# v# " << verts.size() << " f# "  << faces_verts.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
  return (int)verts.size();
}

int Model::nfaces() {
  return (int)faces_verts.size();
}

std::vector<int> Model::face(int idx) {
  return faces_verts[idx];
}

std::vector<int> Model::faces_vt(int idx) {
  return faces_texts[idx];
}

Vec3f Model::vert(int i) {
  return verts[i];
}

Vec3f Model::texture_vert(int i) {
  return textures[i];
}

TGAColor Model::diffuse(Vec2f texture_coords) {
  return diffuse_map.get(
    texture_coords.x * diffuse_map.get_width(),
    texture_coords.y * diffuse_map.get_height()
  );
}

void Model::load_textures(const char* texts_file, TGAImage &img) {
  std::cerr << "Texture file " << texts_file << " loading " << (img.read_tga_file(texts_file) ? "Done." : "failed") << std::endl;
  img.flip_vertically();
}
