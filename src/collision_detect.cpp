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
    findCollisionCandidates(root, &F);

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

std::vector<std::pair<int, int>>* CollisionDetect::findCollisionCandidates(BVHNode *root, Eigen::MatrixXi *F) {

  // Create candidates vector (pairs of triangle indices)
  std::vector<std::pair<int, int>> *candidates = 
    new std::vector<std::pair<int, int>>();

  // Get all vertices from BVHNode root
  Eigen::MatrixXd *V = root->allV;

  // Find candidates for each triangle in mesh
  for (int i = 0; i < F->rows(); i++) {

    // Get bounding box for current triangle
    Eigen::VectorXi currTri = F->row(i);
    Eigen::MatrixXd triPoints = root->triangleToPoints(currTri);
    BoundingBox *currTriBox = new BoundingBox(triPoints);

    // Find collision candidates by crawling tree
    std::queue<BVHNode*> *intersectQueue = new std::queue<BVHNode*>();
    intersectQueue->push(root);
    while (!intersectQueue->empty()) {
      // Get first element
      BVHNode *curr = intersectQueue->front();
      intersectQueue->pop();

      // Check if leaf. Add as candidate (if valid)
      if (curr->isLeaf()) {

        // Get triangle candidate
        Eigen::Vector4i otherTri = curr->triangle;
        int otherInd = otherTri(3);

        // Check if ind of otherInd > mine. Prevents candidates dupes
        if (otherInd > i) {
          // Make sure triangles aren't neighbors (share vert)
          if (!triNeighbors(curr, currTri, otherTri)) {
            // Add triangle indices to candidates list
            candidates->push_back(std::pair<int, int>(i, otherInd));
          }
        }
        continue;
      }

      // If not leaf, check if curr triangle intersects with left/right child and crawl as necessary
      BVHNode* currLeft = curr->left;
      BVHNode* currRight= curr->right;
      if (currTriBox->intersectsWith(currLeft->boundingBox)) {
        intersectQueue->push(currLeft);
      }
      if (currTriBox->intersectsWith(currRight->boundingBox)) {
        intersectQueue->push(currRight);
      }
    }
  }
  return candidates; 
}

std::vector<std::pair<int, int>>* CollisionDetect::findCollisionsFromCandidates(std::vector<std::pair<int, int>> *candidates, Eigen::MatrixXd *V, Eigen::MatrixXi *F) {
  //TODO
  return new std::vector<std::pair<int, int>>();
}

bool CollisionDetect::triNeighbors(BVHNode *node, Eigen::VectorXi triOne, Eigen::VectorXi triTwo) {
  // Returns true iff triangles share at least 1 point
    // NOTE: node doesn't matter, only needs triangleToPoints method of the node

  Eigen::MatrixXd triOnePoints = node->triangleToPoints(triOne);
  Eigen::MatrixXd triTwoPoints = node->triangleToPoints(triTwo);

  // Check every pair of points for equality
  for (int i = 0; i < triOnePoints.rows(); i++) {
    for (int j = 0; j < triTwoPoints.rows(); j++) {
      if (triOnePoints.row(i).isApprox(triTwoPoints.row(j))) {
        return true;
      }
    }
    
  }
  return false;
}
