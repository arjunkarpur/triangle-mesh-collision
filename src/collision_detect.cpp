#include "collision_detect.h"
#include "bvh.h"
#include <iostream>
#include <queue>

void inspectTree(BVHNode *root) {
  int level = 1;
  std::queue<BVHNode*> *nodes = new std::queue<BVHNode*>();
  std::queue<BVHNode*> *nextLevel = new std::queue<BVHNode*>();

  nodes->push(root);

  std::cout << level << std::endl;
  while (true) {
    if (nodes->empty()) {
      if (nextLevel->empty()) {
        break;
      }
      nodes = nextLevel;
      nextLevel = new std::queue<BVHNode*>();
      level += 1;
      std::cout << "============" << std::endl;
      std::cout << level << std::endl;
    }

    std::cout << "-----" << std::endl;
    BVHNode *curr = nodes->front();
    nodes->pop();

    std::cout << "BB: " << curr->boundingBox->getMinMax() << std::endl;
    std::cout << "LEAF? " << curr->isLeaf() << std::endl;
    if (curr->isLeaf()) {
        std::cout << "TRI: " << curr->triangle << std::endl;
    
    }

    if (curr->left != nullptr) {
      nextLevel->push(curr->left);
    }
    if (curr->right != nullptr) {
      nextLevel->push(curr->right);
    }
  }
}

bool CollisionDetect::hasCollision(Eigen::MatrixXd V, Eigen::MatrixXi F) {

  // Construct the BVH data structure
  BVHNode *root = loadMeshToBVH(V, F);
  inspectTree(root);
    
  //TODO: 
  //  for each triangle, find collision candidates in BVT

  //TODO: 
  //  inspect candidates further and find all collisions

  return true;
}

BVHNode* CollisionDetect::loadMeshToBVH(Eigen::MatrixXd V, Eigen::MatrixXi F) {

  // Get list of indices for triangles and append to triangle data
  Eigen::VectorXi indices(F.rows());
  for (int i = 0; i < F.rows(); i++) {
      indices[i] = i;
  }
  Eigen::MatrixXi indexF(F.rows(), F.cols() + 1);
  indexF << F, indices;

  //TODO: bug - creates a node always, not a root in special case when only 1 triangle
  BVHNode *root = new BVHNode(&V, indexF);
  return root;
}
