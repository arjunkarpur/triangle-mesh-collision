#ifndef __BOUNDING_BOX_H__
#define __BOUNDING_BOX_H__

#include <Eigen/Core>

class BoundingBox {
  public:
    Eigen::MatrixXd minMax;

    BoundingBox(Eigen::MatrixXd points);
    ~BoundingBox(){};
    Eigen::MatrixXd getMinMax();
    Eigen::MatrixXd getCorners();
    bool intersectsWith(BoundingBox otherBox);
};

#endif
