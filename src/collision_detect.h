#ifndef __COLLISION_DETECT_H__
#define __COLLISION_DETECT_H__

#include <Eigen/Core>
#include "bvh.h"

class CollisionDetect{
 
  public:   
    CollisionDetect(){};
    ~CollisionDetect(){};
    bool hasCollision(Eigen::MatrixXd V, Eigen::MatrixXi F);

  private:
    BVHNode* loadMeshToBVH(Eigen::MatrixXd V, Eigen::MatrixXi F);
    std::vector<std::pair<int, int>>* findCollisionCandidates(BVHNode* root, Eigen::MatrixXi *F);
    std::vector<std::pair<int, int>>* findCollisionsFromCandidates(std::vector<std::pair<int, int>>* candidates, Eigen::MatrixXd *V, Eigen::MatrixXi *F);
    bool triNeighbors(BVHNode *node, Eigen::VectorXi triOne, Eigen::VectorXi triTwo);
};

#endif
