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
    std::vector<std::pair<int, int>> findCollisions(Eigen::MatrixXd *V, Eigen::MatrixXi *F);

  private:
    std::vector<Eigen::MatrixXd>* getAllTrianglePoints(Eigen::MatrixXd *V, Eigen::MatrixXi *F);
    BVHNode* loadMeshToBVH(Eigen::MatrixXd *V, Eigen::MatrixXi *F, std::vector<Eigen::MatrixXd> *allTriPoints);
    std::vector<std::pair<int, int>>* findCollisionCandidates(BVHNode* root, Eigen::MatrixXd *V, Eigen::MatrixXi *F, std::vector<Eigen::MatrixXd> *allTriPoints);
    std::vector<std::pair<int, int>>* findCollisionsFromCandidates(std::vector<std::pair<int, int>>* candidates, Eigen::MatrixXd *V, Eigen::MatrixXi *F, std::vector<Eigen::MatrixXd> *allTriPoints);
    bool triNeighbors(int indOne, int indTwo, std::vector<Eigen::MatrixXd> *allTriPoints);
    bool trianglesIntersect(Eigen::MatrixXd *pointsOne, Eigen::MatrixXd *pointsTwo);
    bool edgeTriangleIntersect(Eigen::Vector3d v0, Eigen::Vector3d v1, Eigen::Vector3d t0, Eigen::Vector3d t1, Eigen::Vector3d t2);
};

#endif
