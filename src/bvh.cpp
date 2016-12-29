/*
  Implementation fo Bounding Volume Heirarchy (BVH) data struct
*/

#include "bvh.h"
#include <algorithm>
#include <assert.h>
#include <vector>
#include <iostream>

BVHNode::BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris) {
  this->allV = allV;
  left = nullptr;
  right = nullptr;
  buildNode(nodeTris);
}

bool sortMinInd(std::pair<double, int> i, std::pair<double, int> j) {
    return (i.first < j.first);
}

void BVHNode::buildNode(Eigen::MatrixXi nodeTris) {
  // If node is a leaf, save triangle/bounding box and exit 
  if (nodeTris.rows() == 1) {
    triangle = nodeTris.row(0);
    Eigen::MatrixXd points = triangleToPoints(triangle);
    boundingBox = new BoundingBox(points);
    return;
  } else {
    triangle = Eigen::RowVector4i(-1, -1, -1, -1);
  }

  // Compute bouding box for triangles (save to instance var)
  boundingBox = findBoundingBoxSet(nodeTris);

  // Determine longest length of bounding box, split into 2
  Eigen::MatrixXd minMax = boundingBox->getMinMax();

  double xDiff = minMax(1,0) - minMax(0,0);
  double yDiff = minMax(1,1) - minMax(0,1);
  double zDiff = minMax(1,2) - minMax(0,2);
  double maxDiff = std::max(xDiff, std::max(yDiff, zDiff));
  std::vector<std::pair<double, int>> minInd;
  for (int i = 0; i < nodeTris.rows(); i++) {
    
    BoundingBox *currTriBox= 
      new BoundingBox(triangleToPoints(nodeTris.row(i)));

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
    Eigen::RowVectorXi currTriangle = nodeTris.row(curr.second);
    firstTriangles.block<1,4>(i,0) = currTriangle;
  }
  for (int i = 0; i < secSplit.size(); i++) {
    std::pair<double, int> curr = secSplit[i];
    Eigen::RowVectorXi currTriangle = nodeTris.row(curr.second);
    secTriangles.block<1,4>(i,0) = currTriangle;
  }

  // Create left and right children
  left = new BVHNode(allV, firstTriangles);
  right = new BVHNode(allV, secTriangles);

  return;
}

Eigen::MatrixXd BVHNode::triangleToPoints(Eigen::Vector4i triangle) {
  Eigen::RowVector3d p1 = (*allV).block<1,3>(triangle(0), 0);
  Eigen::RowVector3d p2 = (*allV).block<1,3>(triangle(1), 0);
  Eigen::RowVector3d p3 = (*allV).block<1,3>(triangle(2), 0);
  Eigen::MatrixXd points(3, 3);
  points << p1, p2, p3;
  return points;
}

BoundingBox* BVHNode::findBoundingBoxSet(Eigen::MatrixXi triangles) {
  Eigen::MatrixXd points(triangles.rows()*3, 3);
  for (int i = 0; i < triangles.rows(); i++) {
      points.block<3, 3>(i*3, 0) = triangleToPoints(triangles.row(i));
  }
  return (new BoundingBox(points));
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
