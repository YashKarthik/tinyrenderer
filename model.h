#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<Vec3f> verts{};
  std::vector<Vec3f> textures{};
  std::vector<Vec3f> norms{};

	std::vector<std::vector<int>> faces_verts{};
  std::vector<std::vector<int>> faces_texts{};
  std::vector<std::vector<int>> faces_norms{};
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	Vec3f vert(int i);
  Vec3f texture_vert(int i);
	std::vector<int> face(int idx);
	std::vector<int> faces_vt(int idx);
};

#endif //__MODEL_H__
