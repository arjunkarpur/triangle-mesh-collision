#ifndef __BVH_H__
#define __BVH_H__

#include <Eigen/Core>
#include "bounding_box.h"
#include <queue>

/* 
  Declaration of Bounding Volume Heirarchy (BVH) data struct
*/

class BVHNode {
  public:
    BVHNode *left;
    BVHNode *right;
    Eigen::RowVector4i triangle;
    BoundingBox *boundingBox;

    BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *nodeTris);
    ~BVHNode() {};
    bool isLeaf() { return (left == nullptr && right == nullptr); };
    void inspectTree();
    static Eigen::MatrixXd triangleToPoints(Eigen::MatrixXd *points, Eigen::VectorXi triangle);

  private:
    void buildNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *nodeTris);
    BoundingBox* findBoundingBoxSet(Eigen::MatrixXd *allV, Eigen::MatrixXi *triangles);
};

#endif
