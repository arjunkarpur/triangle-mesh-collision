#include "collision_detect.h"
#include "bvh.h"

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

    //TODO: creates a node always, not a root in special case when only 1 triangle
    BVHNode *root = new BVHNode(&V, F);
    return root;
}
