#include "geometry.h"
#include "model.h"

extern const int width;
extern const int height;
extern const int depth;
extern Vec3f light_dir;
extern Vec3f eye;
extern Vec3f center;
extern Model *model;

extern  Matrix Projection;
extern  Matrix Viewport;
extern  Matrix View;

void triangle(Vec3f *pts, Vec3f *texture_pts, Vec3f *vertex_normals, float z_buffer[], TGAImage &image);
Vec3f barycentric(Vec3f *pts, Vec3f P);


void lookat(Vec3f eye, Vec3f center, Vec3f up);
void viewport(int x, int y, int w, int h);
void projection(float coeff=0.f);
