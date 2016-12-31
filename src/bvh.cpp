/*
  Implementation fo Bounding Volume Heirarchy (BVH) data struct
*/

#include "bvh.h"
#include <algorithm>
#include <assert.h>
#include <vector>
#include <iostream>

BVHNode::BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *nodeTris) {
  left = nullptr;
  right = nullptr;
  buildNode(allV, nodeTris);
}

bool sortMinInd(std::pair<double, int> i, std::pair<double, int> j) {
    return (i.first < j.first);
}

void BVHNode::buildNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *nodeTris) {
  // If node is a leaf, save triangle/bounding box and exit 
  if (nodeTris->rows() == 1) {
    triangle = nodeTris->row(0);
    Eigen::MatrixXd points = BVHNode::triangleToPoints(allV, triangle);
    boundingBox = new BoundingBox(points);
    return;
  } else {
    triangle = Eigen::RowVector4i(-1, -1, -1, -1);
  }

  // Compute bouding box for triangles (save to instance var)
  boundingBox = findBoundingBoxSet(allV, nodeTris);

  // Determine longest length of bounding box, split into 2
  Eigen::MatrixXd minMax = boundingBox->getMinMax();

  double xDiff = minMax(1,0) - minMax(0,0);
  double yDiff = minMax(1,1) - minMax(0,1);
  double zDiff = minMax(1,2) - minMax(0,2);
  double maxDiff = std::max(xDiff, std::max(yDiff, zDiff));
  std::vector<std::pair<double, int>> minInd;


  for (int i = 0; i < nodeTris->rows(); i++) {
    BoundingBox *currTriBox= 
      new BoundingBox(BVHNode::triangleToPoints(allV, nodeTris->row(i)));

    // Get the min value to sort on 
    //   (depending on longest dimension of bounding box)
    double minVal = 0;
    if (maxDiff == xDiff) { //minX
      minVal = (currTriBox->getMinMax())(0,0);
    } else if (maxDiff == yDiff) { //minY
      minVal = (currTriBox->getMinMax())(0,1);
    } else if (maxDiff == zDiff) { //minZ
      minVal = (currTriBox->getMinMax())(0,2);
    } else {
      // Should never happen! Problem with equality of doubles
      assert(false);
    }
    minInd.push_back(std::pair<double,int>(minVal, i));
  }

  // Sort minInd and partition triangles into 2 matrices
  std::sort(minInd.begin(), minInd.end(), sortMinInd);
  int half = minInd.size()/2;
  std::vector<std::pair<double, int>> firstSplit(minInd.begin(), minInd.begin()+half);
  std::vector<std::pair<double, int>> secSplit(minInd.begin()+half, minInd.end());

  // Copy triangles into matrices to send to child nodes
  Eigen::MatrixXi firstTriangles(firstSplit.size(), 4);
  Eigen::MatrixXi secTriangles(secSplit.size(), 4);
  for (int i = 0; i < firstSplit.size(); i++) {
    std::pair<double, int> curr = firstSplit[i];
    Eigen::RowVectorXi currTriangle = nodeTris->row(curr.second);
    firstTriangles.block<1,4>(i,0) = currTriangle;
  }
  for (int i = 0; i < secSplit.size(); i++) {
    std::pair<double, int> curr = secSplit[i];
    Eigen::RowVectorXi currTriangle = nodeTris->row(curr.second);
    secTriangles.block<1,4>(i,0) = currTriangle;
  }

  // Create left and right children
  left = new BVHNode(allV, &firstTriangles);
  right = new BVHNode(allV, &secTriangles);

  return;
}

Eigen::MatrixXd BVHNode::triangleToPoints(Eigen::MatrixXd *points, Eigen::VectorXi triangle) {
  Eigen::RowVector3d p1 = points->block<1,3>(triangle(0), 0);
  Eigen::RowVector3d p2 = points->block<1,3>(triangle(1), 0);
  Eigen::RowVector3d p3 = points->block<1,3>(triangle(2), 0);
  Eigen::MatrixXd triPoints(3, 3);
  triPoints << p1, p2, p3;
  return triPoints;
}

BoundingBox* BVHNode::findBoundingBoxSet(Eigen::MatrixXd *allV, Eigen::MatrixXi *triangles) {
  Eigen::MatrixXd points(triangles->rows()*3, 3);
  for (int i = 0; i < triangles->rows(); i++) {
      points.block<3, 3>(i*3, 0) = BVHNode::triangleToPoints(allV, triangles->row(i));
  }
  return (new BoundingBox(points));
}

void BVHNode::inspectTree() {
  int level = 1;
  std::queue<BVHNode*> *nodes = new std::queue<BVHNode*>();
  std::queue<BVHNode*> *nextLevel = new std::queue<BVHNode*>();

  nodes->push(this);

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


/*
    how to construct bvt:

- for each node (given a set of triangles):
    - check if leaf
    - construct bounding box. save
    - find longest length
    - divide bounding box by 2
    - partition triangles from set into the 2 regions
        - for each triangle in set, create bounding box and check collision with both
    - set left, right = recursive call
*/
