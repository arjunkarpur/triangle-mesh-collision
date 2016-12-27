#ifndef __COLLISION_DETECT_H__
#define __COLLISION_DETECT_H__

#include <Eigen/Core>

class CollisionDetect{
 
  public:   
    CollisionDetect(){};
    ~CollisionDetect(){};
    bool hasCollision(Eigen::MatrixXd V, Eigen::MatrixXi F);
};

#endif
