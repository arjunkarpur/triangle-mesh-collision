#ifndef __BVH_H__
#define __BVH_H__

#include <Eigen/Core>
#include "bounding_box.h"
#include <queue>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <iostream>

/* 
  Declaration of Bounding Volume Heirarchy (BVH) data struct
*/

class BVHNode {
  public:
    BVHNode *left;
    BVHNode *right;
    Eigen::RowVector4i triangle;
    BoundingBox *boundingBox;

    BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *allF, std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds);
    ~BVHNode() {};
    bool isLeaf() { return (left == nullptr && right == nullptr); };
    void inspectTree();
    static Eigen::MatrixXd triangleToPoints(Eigen::MatrixXd *points, Eigen::VectorXi triangle);

  private:
    void buildNode(Eigen::MatrixXd *allV, Eigen::MatrixXi *allF, std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds);
    BoundingBox* findBoundingBoxSet(std::vector<Eigen::MatrixXd> *allTriPoints, std::vector<int> triInds);
};

#endif
