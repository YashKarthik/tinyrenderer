#include "geometry.h"
#include "tgaimage.h"

Vec3f barycentric(Vec3f *pts, Vec3f P);

void lookat(Vec3f eye, Vec3f center, Vec3f up);
void viewport(int x, int y, int w, int h);
void projection(float coeff = 0.f);

extern const int width;
extern const int depth;
extern const int height;

extern Matrix ModelView;
extern Matrix Viewport;
extern Matrix Projection;

struct IShader {
    virtual ~IShader() = default;
    virtual Vec3f vertex(int iface, int nthvert) = 0;
    virtual bool fragment(Vec3f bar, TGAColor &color) = 0;
};

void triangle(Vec3f *pts, IShader &shader, TGAImage &image, float z_buffer[]);
