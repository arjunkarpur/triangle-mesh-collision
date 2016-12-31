#include "collision_detect.h"

std::vector<std::pair<int, int>> CollisionDetect::findCollisions(Eigen::MatrixXd V, Eigen::MatrixXi F) {

  // Construct the BVH data structure
  BVHNode *root = loadMeshToBVH(V, F);
  //root->inspectTree(); 
    
  // Find collision candidates using BVH
  std::vector<std::pair<int, int>> *candidates = 
    findCollisionCandidates(root, &V, &F);
  std::cout << "CANDIDATES: " << candidates->size() << std::endl;

  // Inspect candidates further and find all collisions
  std::vector<std::pair<int, int>> *collisions = 
    findCollisionsFromCandidates(candidates, &V, &F);

  return *collisions;
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

  // Create candidates vector (pairs of triangle indices)
  std::vector<std::pair<int, int>> *candidates = 
    new std::vector<std::pair<int, int>>();

  // Get all vertices from BVHNode root
  //Eigen::MatrixXd *V = root->allV;

  // Find candidates for each triangle in mesh
  for (int i = 0; i < F->rows(); i++) {

    // Get bounding box for current triangle
    Eigen::VectorXi currTri = F->row(i);
    Eigen::MatrixXd triPoints = BVHNode::triangleToPoints(V, currTri);
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
          if (!triNeighbors(V, currTri, otherTri)) {
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
  
  // Vector of confirmed collisions to return
  std::vector<std::pair<int, int>> *collisions = 
    new std::vector<std::pair<int, int>>();

  for (int i = 0; i < candidates->size(); i++) {

    // Get points for current candidate triangles
    std::pair<int, int> curr = candidates->at(i);
    Eigen::VectorXi triOne = F->row(curr.first);
    Eigen::VectorXi triTwo = F->row(curr.second);
    Eigen::MatrixXd pointsOne = BVHNode::triangleToPoints(V, triOne);
    Eigen::MatrixXd pointsTwo = BVHNode::triangleToPoints(V, triTwo);

    // Check if true collision
    if (trianglesIntersect(pointsOne, pointsTwo)) {
      collisions->push_back(curr);
    }
  }
  return collisions;
}

bool CollisionDetect::triNeighbors(Eigen::MatrixXd *V, Eigen::VectorXi triOne, Eigen::VectorXi triTwo) {
  // Returns true iff triangles share at least 1 point

  Eigen::MatrixXd triOnePoints = BVHNode::triangleToPoints(V, triOne);
  Eigen::MatrixXd triTwoPoints = BVHNode::triangleToPoints(V, triTwo);

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

bool CollisionDetect::trianglesIntersect(Eigen::MatrixXd pointsOne, Eigen::MatrixXd pointsTwo) {

  // Pull out triangle 1's vertices into vectors
  Eigen::Vector3d t1Zero = pointsOne.row(0);
  Eigen::Vector3d t1One = pointsOne.row(1);
  Eigen::Vector3d t1Two = pointsOne.row(2);

  // Pull out triangle 2's vertices into vectors
  Eigen::Vector3d t2Zero = pointsTwo.row(0);
  Eigen::Vector3d t2One = pointsTwo.row(1);
  Eigen::Vector3d t2Two = pointsTwo.row(2);

  // Check triangle 1's edges onto triangle 2
  if (edgeTriangleIntersect(t1Zero, t1One, pointsTwo) ||
      edgeTriangleIntersect(t1One, t1Two, pointsTwo) ||
      edgeTriangleIntersect(t1Two, t1Zero, pointsTwo)) {
    return true;
  }

  // Check triangle 2's edges onto triangle 1
  if (edgeTriangleIntersect(t2Zero, t2One, pointsOne) ||
      edgeTriangleIntersect(t2One, t2Two, pointsOne) ||
      edgeTriangleIntersect(t2Two, t2Zero, pointsOne)) {
    return true;
  }

  // No edge-face intersections, so triangles don't intersect
  return false;
}

bool isApprox(double val, double target) {
  double epsilon = .000000001;
  return ( std::abs(val-target) < epsilon );
}

bool CollisionDetect::edgeTriangleIntersect(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::MatrixXd trianglePoints) {

  /*
    Parametric form of line:
  L(t) = v0 + t*(v1-v0)

    Parametric form of plane:
  P(u,v) = t0 + u*(t1-t0) + v*(t2-t0)

    To find point of intersection, find t, u, v where
  L(t) = P(u,v)

  - If 0 <= t <= 1, then line seg intersects plane
  - If 0 <= u,v <= 1 and u+v <= 1, then line seg intersects triangle
  */

  // Put triangle points into vectors
  Eigen::Vector3d t0 = trianglePoints.row(0);
  Eigen::Vector3d t1 = trianglePoints.row(1);
  Eigen::Vector3d t2 = trianglePoints.row(2);

  // Check if line segment and plane triangle lies in are parallel
  Eigen::VectorXd triPlaneNormal = (t1 - t0).cross(t2-t0);
  Eigen::VectorXd lineVector = (v1 - v0);
  bool parallel = isApprox(triPlaneNormal.dot(lineVector), 0);
  if (parallel) {
    //TODO: check if v0 or v1 are in plane (segment inside tri)
    //  right now, assumes not
    return false;
  } 
  
  // Solve for t, u, v
  Eigen::MatrixXd directions = (Eigen::MatrixXd(3,3));
  directions << (v0-v1), (t1-t0), (t2-t0);
  Eigen::VectorXd consts = v0 - t0;
  Eigen::VectorXd sols = (directions.inverse())*consts;

  double t = sols(0);
  double u = sols(1);
  double v = sols(2);

  if (t >= 0 && t <= 1) {
    // If true, line segment intersects with plane

    if (u >= 0 && u <= 1 &&
        v >= 0 && v <= 1 &&
        u + v <= 1) {
      // If true, intersection inside triangle
      return true;
    }
  }
  return false;
}

/*
TODO: more robust than 6 edge/face tests (current implementation)
  should finish implementation later

// Implementation of 'A Fast Triangle-Triangle Intersection Test' [Moller 1997]
bool CollisionDetect::trianglesIntersect(Eigen::MatrixXd pointsOne, Eigen::MatrixXd pointsTwo) {

  // Pull out triangle 1's vertices into vectors
  Eigen::Vector3d t1Zero = pointsOne.row(0);
  Eigen::Vector3d t1One = pointsOne.row(1);
  Eigen::Vector3d t1Two = pointsOne.row(2);

  // Pull out triangle 2's vertices into vectors
  Eigen::Vector3d t2Zero = pointsTwo.row(0);
  Eigen::Vector3d t2One = pointsTwo.row(1);
  Eigen::Vector3d t2Two = pointsTwo.row(2);

  // Early rejection test 1 
  //   (t1 entirely on one side of t2's plane)
  Eigen::VectorXd n2 = (t2One - t2Zero).cross(t2Two - t2Zero);
  double d2 = (-1*(n2)).dot(t2Zero);
  double d_v1_0 = n2.dot(t1Zero) + d2;
  double d_v1_1 = n2.dot(t1One) + d2;
  double d_v1_2 = n2.dot(t1Two) + d2;
  if ((d_v1_0 > 0 && d_v1_1 > 0 && d_v1_2 > 0) ||
      (d_v1_0 < 0 && d_v1_1 < 0 && d_v1_2 < 0)) {
    return false;
  }

  // Early rejection test 2 
  //   (t2 entirely on one side of t1's plane)
  Eigen::VectorXd n1 = (t1One - t1Zero).cross(t1Two - t1Zero);
  double d1 = (-1*(n2)).dot(t1Zero);
  double d_v2_0 = n1.dot(t2Zero) + d1;
  double d_v2_1 = n1.dot(t2One) + d1;
  double d_v2_2 = n1.dot(t2Two) + d1;
  if ((d_v2_0 > 0 && d_v2_1 > 0 && d_v2_2 > 0) ||
      (d_v2_0 < 0 && d_v2_1 < 0 && d_v2_2 < 0)) {
    return false;
  }

  // Triangles are co-planar. Perform co-planar tests
  if ((isApprox(d_v1_0, 0) && isApprox(d_v1_1, 0) && isApprox(d_v1_2, 0)) ||
      (isApprox(d_v2_0, 0) && isApprox(d_v2_1, 0) && isApprox(d_v2_2, 0))) {
    //TODO: coplanar tests
  }

  // Triangles must intersect on some line L (intersection of the two planes)
  //   See if intervals for each's intersection on L overlap.
  //TODO 

  return true;
}
*/
