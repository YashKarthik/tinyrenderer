#include <vector>
#include <cassert>
#include "geometry.h"

Matrix::Matrix(int r, int c):
  m(std::vector<std::vector<float>>(r, std::vector<float>(c, 0.f))),
  rows{r},
  cols{c}
{}

Matrix::Matrix(Vec3f v):
  m(std::vector<std::vector<float>>(4, std::vector<float>(1, 0.f))),
  rows{4},
  cols{1}
{
  m[0][0] = v.x;
  m[1][0] = v.y;
  m[2][0] = v.z;
  m[3][0] = 1.f;
}

inline int Matrix::nrows() {return rows; }
inline int Matrix::ncols() {return cols; }

Matrix Matrix::identity(int dimensions) {
  Matrix I(dimensions, dimensions);

  for (int i{0}; i < dimensions; i++) {
    for (int j{0}; j < dimensions; j++) {
      I[i][j] = (i == j) ? 1.f : 0.f;
    }
  }

  return I;
}

std::vector<float>& Matrix::operator[](const int i) {
  assert(i >= 0 && i < rows);

  return m[i];
}

Matrix Matrix::operator*(const Matrix& a) {
  assert(cols == a.rows);
  Matrix result(rows, a.cols);

  for (int i{0}; i < rows; i++) {

    for (int j{0}; j < a.cols; j++) {
      result.m[i][j] = 0.f;

      for (int k{0}; k < cols; k++) {
        result.m[i][j] += m[i][k]*a.m[k][j];
      }
    }
  }
  return result;
}

Matrix Matrix::transpose() {
  Matrix result(cols, rows);

  for (int i{0}; i < rows; i++) {
    for (int j{0}; j < cols; j++) {
      result[j][i] = m[i][j];
    }
  }

  return result;
}

Matrix Matrix::inverse() {
  assert(rows==cols);
  // augmenting the square matrix with the identity matrix of the same dimensions a => [ai]
  Matrix result(rows, cols*2);
  for(int i=0; i<rows; i++)
    for(int j=0; j<cols; j++)
      result[i][j] = m[i][j];
  for(int i=0; i<rows; i++)
    result[i][i+cols] = 1;
  // first pass
  for (int i=0; i<rows-1; i++) {
    // normalize the first row
    for(int j=result.cols-1; j>=0; j--)
      result[i][j] /= result[i][i];
    for (int k=i+1; k<rows; k++) {
      float coeff = result[k][i];
      for (int j=0; j<result.cols; j++) {
        result[k][j] -= result[i][j]*coeff;
      }
    }
  }
  // normalize the last row
  for(int j=result.cols-1; j>=rows-1; j--)
    result[rows-1][j] /= result[rows-1][rows-1];
  // second pass
  for (int i=rows-1; i>0; i--) {
    for (int k=i-1; k>=0; k--) {
      float coeff = result[k][i];
      for (int j=0; j<result.cols; j++) {
        result[k][j] -= result[i][j]*coeff;
      }
    }
  }
  // cut the identity matrix back
  Matrix truncate(rows, cols);
  for(int i=0; i<rows; i++)
    for(int j=0; j<cols; j++)
      truncate[i][j] = result[i][j+cols];
  return truncate;
}

std::ostream& operator<<(std::ostream& s, Matrix& m) {
  for (int i=0; i<m.nrows(); i++)  {
    for (int j=0; j<m.ncols(); j++) {
      s << m[i][j];
      if (j<m.ncols()-1) s << "\t";
    }
    s << "\n";
  }
  return s;
}

Vec3f Matrix::to_Vec3f() {
  return Vec3f(
    m[0][0]/m[3][0],
    m[1][0]/m[3][0],
    m[2][0]/m[3][0]
  );
}
