#include <iostream>
#include <queue>
#include "collision_detect.h"
#include "bvh.h"
#include "bounding_box.h"

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
  //inspectTree(root); UNCOMMENT TO PRINT OUT TREE
    
  // Find collision candidates using BVH
  std::vector<std::pair<int, int>> *candidates = 
    findCollisionCandidates(root, &V, &F);

  // Inspect candidates further and find all collisions
  std::vector<std::pair<int, int>> *collisions = 
    findCollisionsFromCandidates(candidates, &V, &F);

  //TODO: return more information?
  return (!collisions->empty());
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

std::vector<std::pair<int, int>>* CollisionDetect::findCollisionCandidates(BVHNode *root, Eigen::MatrixXd *V, Eigen::MatrixXi *F) {
    //TODO

    // don't forget to check neighbors
    return new std::vector<std::pair<int, int>>();
}

std::vector<std::pair<int, int>>* CollisionDetect::findCollisionsFromCandidates(std::vector<std::pair<int, int>> *candidates, Eigen::MatrixXd *V, Eigen::MatrixXi *F) {
    //TODO
    return new std::vector<std::pair<int, int>>();
}
