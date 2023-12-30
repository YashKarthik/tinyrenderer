#include "geometry.h"
#include "model.h"

extern const int width;
extern const int height;
extern const int depth;
extern Vec3f light_dir;
extern Vec3f eye;
extern Vec3f center;
extern Model *model;

void triangle(Vec3f *pts, Vec3f *texture_pts, Vec3f *vertex_normals, float z_buffer[], TGAImage &image);
Vec3f barycentric(Vec3f *pts, Vec3f P);
