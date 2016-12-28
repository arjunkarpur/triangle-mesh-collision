#ifndef __BVH_H__
#define __BVH_H__

#include <Eigen/Core>

/* 
  Declaration of Bounding Volume Heirarchy (BVH) data struct
*/

class BVHNode {
  public:
    BVHNode *left;
    BVHNode *right;
    //TODO: maintain bounding box?

    BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris);
    ~BVHNode() {};
    bool isLeaf() { return false; };

  private:
    Eigen::MatrixXd *allV;

    void buildNode(Eigen::MatrixXi nodeTris);
};

class BVHLeaf : public BVHNode {
  public:
    //TODO: maintain triangle?

    bool isLeaf() { return true; };
};

#endif
