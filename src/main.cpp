#include <igl/viewer/Viewer.h>
#include "collision_detect.h"
#include "mesh.h"
#include <string>
#include <iostream>

TriangleMesh getTriangleMesh(std::string file) {
  // Inline mesh of a cube

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  std::string fileExt = file.substr(file.find_last_of(".") + 1);
  std::string meshFilepath = "../meshes/" + file;

  if (fileExt == "off") {
    igl::readOFF(meshFilepath,V,F);
  } else {
    //TODO: other file types
  }

  if (V.rows() == 0) {
    std::cout << "Mesh not found: " + meshFilepath << std::endl;
    exit(-1);
  }

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
  
  // Get input mesh file
  if (argc < 2) {
    std::cout << "Input file name/ext as cmd line argument" << std::endl;
    exit(-1);
  }
  std::string file = argv[1];
  TriangleMesh mesh = getTriangleMesh(file);

  // Collision detector
  CollisionDetect *cd = new CollisionDetect();
  std::cout << "HAS COLLISIONS: " << cd->hasCollision(mesh.V, mesh.F) << std::endl;

  // Plot the mesh
  plot_mesh(mesh);
}
