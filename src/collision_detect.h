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
};

#endif
