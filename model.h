#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"
#include "tgaimage.h"

class Model {
private:
	std::vector<Vec3f> verts{};
  std::vector<Vec3f> textures{};
  std::vector<Vec3f> norms{};

	std::vector<std::vector<int>> faces_verts{};
  std::vector<std::vector<int>> faces_texts{};
  std::vector<std::vector<int>> faces_norms{};

  TGAImage diffuse_map{};
  void load_textures(const char* texts_file, TGAImage& img);
public:
	Model(const char *obj_file, const char *texts_file);
	~Model();
	int nverts();
	int nfaces();

	Vec3f vert(int i);
  Vec3f texture_vert(int i);
  Vec3f vert_norm(int i);

	Vec3f vert(int iface, int nthvertex);
  Vec3f texture_vert(int iface, int nthvertex);
  Vec3f vert_norm(int iface, int nthvertex);

	std::vector<int> face_verts(int idx);
	std::vector<int> face_texts(int idx);
	std::vector<int> face_norms(int idx);

  TGAColor diffuse(Vec2f texture_coords);
};

#endif //__MODEL_H__
