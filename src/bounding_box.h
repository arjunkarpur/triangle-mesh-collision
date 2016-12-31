#ifndef __BOUNDING_BOX_H__
#define __BOUNDING_BOX_H__

#include <Eigen/Core>

class BoundingBox {
  public:
    Eigen::MatrixXd minMax;

    BoundingBox() {};
    BoundingBox(Eigen::MatrixXd *points);
    ~BoundingBox(){};
    Eigen::MatrixXd getCorners();
    bool intersectsWith(BoundingBox *otherBox);
};

#endif
