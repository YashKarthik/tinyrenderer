#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "geometry.h"
#include "model.h"
#include <iostream>

Model::Model(
  const char *obj_file,
  const char *texts_file,
  const char *nm_file,
  const char *spec_file
) : verts(), faces_verts() {

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

    } else if (!line.compare(0, 2, "vn")) {
      iss >> trash >> trash;
      Vec3f vn;
      for (int i{0}; i < 10; i++) iss >> vn.raw[i];
      norms.push_back(vn);

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

  load_map(texts_file, diffuse_map);
  load_map(nm_file, normal_map);
  load_map(spec_file, specular_map);

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

std::vector<int> Model::face_verts(int idx) {
  return faces_verts[idx];
}

std::vector<int> Model::face_texts(int idx) {
  return faces_texts[idx];
}

std::vector<int> Model::face_norms(int idx) {
  return faces_norms[idx];
}

Vec3f Model::vert(int i) {
  return verts[i];
}

Vec3f Model::vert(int iface, int nthvertex) {
  return verts[ face_verts(iface)[nthvertex] ];
}

Vec3f Model::texture_vert(int i) {
  return textures[i];
}

Vec3f Model::texture_vert(int iface, int nthvert) {
  return textures[ face_texts(iface)[nthvert] ];
}

Vec3f Model::vert_norm(int i) {
  return norms[i].normalize();
}

Vec3f Model::vert_norm(int iface, int nthvert) {
  return norms[ face_norms(iface)[nthvert] ].normalize();
}

TGAColor Model::diffuse(Vec2f uv) {
  return diffuse_map.get(
    uv.x * diffuse_map.get_width(),
    uv.y * diffuse_map.get_height()
  );
}

Vec3f Model::normal(Vec2f uv) {
  TGAColor c = normal_map.get(
    uv.x * normal_map.get_width(),
    uv.y * normal_map.get_height()
  );

  /* Mapping rgb values to vector space:
   *  1. Normalize rgb into range [0, 1]: float div by 255.
   *  2. Map [0, 1] to [-1 , 1]: x*2 - 1
   *  3. Construct vector.
   *
   * In TGAColor, raw is [ b, g, r, a ], hence the res.raw[2 - i];
   **/

  Vec3f res;
  for (int i{0}; i < 3; i++) {
    res.raw[2 - i] = (float)c.raw[i]/255.f*2.f - 1.f;
  }
  return res;
}

float Model::specular(Vec2f uvf) {
  Vec2i uv(
    uvf.x*specular_map.get_width(),
    uvf.y*specular_map.get_height()
  );

  return specular_map.get(uv.x, uv.y).raw[0]/1.f;
}

void Model::load_map(const char* texts_file, TGAImage &img) {
  std::cerr << "Texture file " << texts_file << " loading " << (img.read_tga_file(texts_file) ? "Loaded textures." : "failed") << std::endl;
  img.flip_vertically();
}
