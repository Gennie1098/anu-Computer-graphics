#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include "Eigen/Eigen"
#include "Eigen/src/Core/Matrix.h"
#include "Triangle.hpp"
#include <cstddef>
#include <iostream>

using namespace Eigen; // namespace Eigen;

// TODO: Implement this function to test if point (x, y) is inside the triangle.
/**
 * @param _v: a pointer to the array of three triangle vertices. The coordiante of
 * vertices is homogenous.
 */
static bool insideTriangle(float x, float y, const Eigen::Vector3f *_v) {
  //create point with given x, y
  Vector2f p(x, y);
  //vertice 1 of triangles
  Vector2f v0(_v[0].x(), _v[0].y());
  //vertice 2 of triangles
  Vector2f v1(_v[1].x(), _v[1].y());
  //vertice 3 of triangles
  Vector2f v2(_v[2].x(), _v[2].y());

  //vector between vertices/edges
  Vector2f e0 = v1 - v0;
  Vector2f e1 = v2 - v1;
  Vector2f e2 = v0 - v2;

  //vector between checking point and vertices
  Vector2f vp0 = p - v0;
  Vector2f vp1 = p - v1;
  Vector2f vp2 = p - v2;

  //cross product of vectors
  float c0 = e0.dot(vp0);
  float c1 = e1.dot(vp1);
  float c2 = e2.dot(vp2);

  //check if point is inside triangle = all dot products >= 0 or <=0
  return (c0 >= 0 && c1 >= 0 && c2 >= 0) || (c0 <= 0 && c1 <= 0 && c2 <= 0);
}

int main(int argc, char const *argv[]) {
  //
  std::vector<Triangle> triangles;

  std::vector<Vector3f> vertices{{0.f, 0.f, 1.f}, {2.f, 0.f, 1.f},
                                 {1.f, 1.f, 1.f}, {3.f, 3.f, 1.0},
                                 {3.f, 5.f, 1.f}, {5.f, 3.f, 1.f}};
  for (size_t i = 0; i < 2; ++i) {
    Triangle t;
    for (size_t j = 0; j < 3; ++j) {
      t.setVertex(j, vertices[i * 3 + j]);
    }
    triangles.push_back(t);
  }
  std::vector<Vector2f> points{{1.0, 0.5}, {4.f, 3.5f}};

  for (auto &point : points) {
    for (auto &t : triangles) {
      if (insideTriangle(point.x(), point.y(), t.v)) {
        std::cout << "Point: (" << point.x() << ", " << point.y()
                  << ") is inside the triangle: " << t << std::endl;
      }
      else{
        std::cout << "Point: (" << point.x() << ", " << point.y()
                  << ") is not inside the triangle: " << t << std::endl;
      }
    }
  }
  return 0;
}