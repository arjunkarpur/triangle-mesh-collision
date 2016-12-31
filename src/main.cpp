#include <igl/viewer/Viewer.h>
#include "collision_detect.h"
#include "mesh.h"
#include <string>
#include <iostream>
#include <igl/readPLY.h>
#include "bvh.h"

TriangleMesh getTriangleMesh(std::string file) {
  // Inline mesh of a cube

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  std::string fileExt = file.substr(file.find_last_of(".") + 1);
  std::string meshFilepath = "../meshes/" + file;

  if (fileExt == "off") {
    igl::readOFF(meshFilepath,V,F);
  } else if (fileExt == "ply") {
    igl::readPLY(meshFilepath,V,F);
  } else {
    // other file types, implement as necessary
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

void visualizeCollisions(TriangleMesh *mesh, std::vector<std::pair<int,int>> collisions) {
    
  for (int i =0; i < collisions.size(); i++) {
    std::cout << "------" << std::endl;
    std::cout << "Collision " << i << "/" << collisions.size() << std::endl;
    int tri1 = collisions.at(i).first;
    int tri2 = collisions.at(i).second;
    std::cout << "TRI INDS: " << tri1 << " " << tri2 << std::endl;

    Eigen::MatrixXd points1 = 
      BVHNode::triangleToPoints(&(mesh->V), mesh->F.row(tri1));

    Eigen::MatrixXd points2 = 
      BVHNode::triangleToPoints(&(mesh->V), mesh->F.row(tri2));

    Eigen::MatrixXd intersectV(6, 3);
    intersectV << points1, points2;
    Eigen::MatrixXi intersectF = (Eigen::MatrixXi(2,3) <<
      0, 1, 2,
      3, 4, 5
    ).finished();
    TriangleMesh m;
    m.V = intersectV;
    m.F = intersectF;
    plot_mesh(m);
  }
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
  std::vector<std::pair<int, int>> collisions = 
    cd->findCollisions(mesh.V, mesh.F);
  std::cout << "COLLISIONS: " << collisions.size() << std::endl;
  
  // Plot the mesh and show collisions
  plot_mesh(mesh);
  visualizeCollisions(&mesh, collisions);
}
