#include "collision_detect.h"
#include "bvh.h"
#include <iostream>

bool CollisionDetect::hasCollision(Eigen::MatrixXd V, Eigen::MatrixXi F) {

  // Construct the BVH data structure
  BVHNode *root = loadMeshToBVH(V, F);
  
  //TODO: 
  //  for each triangle, find collision candidates in BVT

  //TODO: 
  //  inspect candidates further and find all collisions

  return true;
}

BVHNode* CollisionDetect::loadMeshToBVH(Eigen::MatrixXd V, Eigen::MatrixXi F) {

  // TODO: Slow?
  Eigen::VectorXi indices(F.rows());
  for (int i = 0; i < F.rows(); i++) {
      indices[i] = i;
  }
  Eigen::MatrixXi indexF(F.rows(), F.cols() + 1);
  indexF << F, indices;

  //TODO: creates a node always, not a root in special case when only 1 triangle
  BVHNode *root = new BVHNode(&V, indexF);
  return root;
}
