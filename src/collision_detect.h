#ifndef __COLLISION_DETECT_H__
#define __COLLISION_DETECT_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <iostream>
#include "bvh.h"
#include "bounding_box.h"

class CollisionDetect{
 
  public:   
    CollisionDetect(){};
    ~CollisionDetect(){};
    std::vector<std::pair<int, int>> findCollisions(Eigen::MatrixXd V, Eigen::MatrixXi F);

  private:
    BVHNode* loadMeshToBVH(Eigen::MatrixXd V, Eigen::MatrixXi F);
    std::vector<std::pair<int, int>>* findCollisionCandidates(BVHNode* root, Eigen::MatrixXd *V, Eigen::MatrixXi *F);
    std::vector<std::pair<int, int>>* findCollisionsFromCandidates(std::vector<std::pair<int, int>>* candidates, Eigen::MatrixXd *V, Eigen::MatrixXi *F);
    bool triNeighbors(Eigen::MatrixXd *V, Eigen::VectorXi triOne, Eigen::VectorXi triTwo);
    bool trianglesIntersect(Eigen::MatrixXd pointsOne, Eigen::MatrixXd pointsTwo);
    bool edgeTriangleIntersect(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::MatrixXd trianglePoints);
};

#endif
