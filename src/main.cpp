#include <igl/viewer/Viewer.h>
#include "collision_detect.h"
#include "mesh.h"

TriangleMesh getTriangleMesh() {
  /*
  // Inline mesh of a cube
  const Eigen::MatrixXd V= (Eigen::MatrixXd(8,3)<<
    0.0,0.0,0.0,
    0.0,0.0,1.0,
    0.0,1.0,0.0,
    0.0,1.0,1.0,
    1.0,0.0,0.0,
    1.0,0.0,1.0,
    1.0,1.0,0.0,
    1.0,1.0,1.0).finished();
  const Eigen::MatrixXi F = (Eigen::MatrixXi(12,3)<<
    1,7,5,
    1,3,7,
    1,4,3,
    1,2,4,
    3,8,7,
    3,4,8,
    5,7,8,
    5,8,6,
    1,5,6,
    1,6,2,
    2,6,8,
    2,8,4).finished().array()-1;
  */

  const Eigen::MatrixXd V = (Eigen::MatrixXd(8,3) <<
    0.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    1.0, 1.0, 0.0,
    2.0, 0.0, 0.0,
    2.0, 1.0, 0.0,
    3.0, 0.0, 0.0,
    3.0, 1.0, 0.0).finished();
  const Eigen::MatrixXi F = (Eigen::MatrixXi(6, 3) <<
    3, 1, 0,
    0, 2, 3,
    5, 3, 2,
    2, 4, 5,
    7, 5, 4,
    4, 6, 7).finished();

  /*
  const Eigen::MatrixXd V = (Eigen::MatrixXd(3,3) <<
    0.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0).finished();
  const Eigen::MatrixXi F = (Eigen::MatrixXi(1, 3) <<
    2, 1, 0).finished();
  */
  TriangleMesh m;
  m.V = V;
  m.F = F;
  return m;
}

void plot_mesh(TriangleMesh mesh) {
  igl::viewer::Viewer viewer;
  viewer.data.set_mesh(mesh.V, mesh.F);
  viewer.data.set_face_based(true);
  viewer.launch();
}

int main(int argc, char *argv[])
{
  
  // Load triangle mesh
  TriangleMesh mesh = getTriangleMesh();

  // Collision detector
  CollisionDetect *cd = new CollisionDetect();
  std::cout << "HAS COLLISIONS: " << cd->hasCollision(mesh.V, mesh.F) << std::endl;

  // Plot the mesh
  plot_mesh(mesh);
}
