/*
  Implementation fo Bounding Volume Heirarchy (BVH) data struct
*/

#include "bvh.h"

BVHNode::BVHNode(Eigen::MatrixXd *allV, Eigen::MatrixXi nodeTris) {
    this->allV = allV;
    buildNode(nodeTris);
}

void BVHNode::buildNode(Eigen::MatrixXi nodeTris) {
    //TODO: 
    //  build the node recursively.
    //  see below for pseudo-code
    return;
}

/*
    how to construct bvt:

- for each node (given a set of triangles):
    - check if leaf
    - construct bounding box. save
    - find longest length
    - divide bounding box by 2
    - partition triangles from set into the 2 regions
        - for each triangle in set, create bounding box and check collision with both
    - set left, right = recursive call
*/
