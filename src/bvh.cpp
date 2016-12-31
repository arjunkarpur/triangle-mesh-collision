/*
  Implementation fo Bounding Volume Heirarchy (BVH) data struct
*/
#include "bvh.h"

BVHNode::BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *allF, std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds) {
  left = nullptr;
  right = nullptr;
  buildNode(allV, allF, allTriPoints, triInds);
}

bool sortMinInd(std::pair<double, int> i, std::pair<double, int> j) {
    return (i.first < j.first);
}

void BVHNode::buildNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *allF, std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds) {
  double begin = std::clock();
  // If node is a leaf, save triangle/bounding box and exit 
  if (triInds.size() == 1) {
    triangle << allF->row(triInds.at(0)), triInds.at(0);
    Eigen::MatrixXd points = BVHNode::triangleToPoints(allV, triangle);
    boundingBox = new BoundingBox(&points);
    double endLeaf = std::clock();
    //std::cout << "leaf constr: " << (endLeaf-begin)/CLOCKS_PER_SEC << std::endl;
    return;
  } 

  // Compute bouding box for triangles (save to instance var)
  boundingBox = findBoundingBoxSet(allV, allTriPoints, triInds);

  // Determine longest length of bounding box, split into 2
  Eigen::MatrixXd minMax = boundingBox->minMax;
  double xDiff = minMax(1,0) - minMax(0,0);
  double yDiff = minMax(1,1) - minMax(0,1);
  double zDiff = minMax(1,2) - minMax(0,2);
  double maxDiff = std::max(xDiff, std::max(yDiff, zDiff));
  std::vector<std::pair<double, int>> minInd;

  for (int i = 0; i < triInds.size(); i++) {
    Eigen::MatrixXd triPoints = 
      allTriPoints->at(triInds.at(i));
    /*
    std::cout << "===" << std::endl;
    std::cout << tmp << std::endl;
    std::cout << "---" << std::endl;
    std::cout << triPoints << std::endl;
    */

    BoundingBox currTriBox(&triPoints);
    // Get the min value to sort on 
    //   (depending on longest dimension of bounding box)
    double minVal = 0;
    if (maxDiff == xDiff) { //minX
      minVal = (currTriBox.minMax)(0,0);
    } else if (maxDiff == yDiff) { //minY
      minVal = (currTriBox.minMax)(0,1);
    } else if (maxDiff == zDiff) { //minZ
      minVal = (currTriBox.minMax)(0,2);
    } else {
      // Should never happen! Problem with equality of doubles
      assert(false);
    }
    minInd.push_back(std::pair<double,int>(minVal, triInds.at(i)));
  }

  // Sort minInd and partition triangles into 2 lists of indices
  std::sort(minInd.begin(), minInd.end(), sortMinInd);
  int half = minInd.size()/2;
  std::vector<int> firstTriangles;
  std::vector<int> secTriangles;
  for (int i = 0; i < half; i++) {
    firstTriangles.push_back(minInd.at(i).second);
  }
  for (int i = half; i < minInd.size(); i++) {
    secTriangles.push_back(minInd.at(i).second);
  }

  double end = std::clock();
  ///std::cout << "node constr: " << (end-begin)/CLOCKS_PER_SEC << std::endl;

  // Create left and right children
  left = new BVHNode(allV, allF, allTriPoints, firstTriangles);
  right = new BVHNode(allV, allF, allTriPoints, secTriangles);

  return;
}

Eigen::MatrixXd BVHNode::triangleToPoints(Eigen::MatrixXd *points, Eigen::VectorXi triangle) {
  Eigen::MatrixXd triPoints(3, 3);
  triPoints << 
    points->block<1,3>(triangle(0),0),
    points->block<1,3>(triangle(1),0),
    points->block<1,3>(triangle(2),0);
  return triPoints;
}

//TODO: fix> make 2nd constructor for bounding box
BoundingBox* BVHNode::findBoundingBoxSet(Eigen::MatrixXd *allV, std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds) {
  Eigen::MatrixXd points(triInds.size()*3, 3);
  for (int i = 0; i < triInds.size(); i++) {
      points.block<3, 3>(i*3, 0) = allTriPoints->at(triInds.at(i));
  }
  return (new BoundingBox(&points));
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

    std::cout << "BB: " << curr->boundingBox->minMax << std::endl;
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
