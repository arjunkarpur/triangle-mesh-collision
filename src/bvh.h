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
    Eigen::RowVector4i triangle;
    Eigen::MatrixXd boundingBox;

    BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris);
    ~BVHNode() {};
    bool isLeaf() { return (left == nullptr && right == nullptr); };

  private:
    Eigen::MatrixXd *allV;
    void buildNode(Eigen::MatrixXi nodeTris);
    Eigen::MatrixXd triangleToPoints(Eigen::Vector4i triangle);
    Eigen::MatrixXd findBoundingBox(Eigen::MatrixXd points);
    Eigen::MatrixXd findBoundingBoxSet(Eigen::MatrixXi triangles);
};

/* unnecessary
class BVHLeaf : public BVHNode {
  public:
    BVHLeaf(Eigen::)
    ~BVHLeaf() {};
    Eigen::RowVector4i triangle;
};
*/

#endif
