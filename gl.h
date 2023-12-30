#include "geometry.h"
#include "tgaimage.h"

void triangle(Vec3f *pts, Vec3f *texture_pts, Vec3f *vertex_normals, float z_buffer[], TGAImage &image);
Vec3f barycentric(Vec3f *pts, Vec3f P);

Matrix lookat(Vec3f eye, Vec3f center, Vec3f up);
Matrix viewport(int x, int y, int w, int h);
