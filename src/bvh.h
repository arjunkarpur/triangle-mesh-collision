#ifndef __BVH_H__
#define __BVH_H__

#include <Eigen/Core>
#include "bounding_box.h"

/* 
  Declaration of Bounding Volume Heirarchy (BVH) data struct
*/

class BVHNode {
  public:
    BVHNode *left;
    BVHNode *right;
    Eigen::MatrixXd *allV;
    Eigen::RowVector4i triangle;
    BoundingBox *boundingBox;

    BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris);
    ~BVHNode() {};
    bool isLeaf() { return (left == nullptr && right == nullptr); };
    Eigen::MatrixXd triangleToPoints(Eigen::VectorXi triangle);

  private:
    void buildNode(Eigen::MatrixXi nodeTris);
    BoundingBox* findBoundingBoxSet(Eigen::MatrixXi triangles);
};

#endif
