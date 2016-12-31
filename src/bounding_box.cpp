#include "bounding_box.h"

BoundingBox::BoundingBox(Eigen::MatrixXd points) {
  Eigen::RowVector3d min = points.colwise().minCoeff();
  Eigen::RowVector3d max = points.colwise().maxCoeff();
  minMax = Eigen::MatrixXd(2,3);
  minMax << min, max;
}

Eigen::MatrixXd BoundingBox::getCorners() {
  //TODO: implement if necessary
  return Eigen::MatrixXd(1,1);
}

bool BoundingBox::intersectsWith(BoundingBox *other) {

  Eigen::MatrixXd otherMinMax = other->minMax;

  bool first = (minMax(0,0) <= otherMinMax(1,0) &&
                minMax(0,1) <= otherMinMax(1,1) &&
                minMax(0,2) <= otherMinMax(1,2));
  bool second = (otherMinMax(0,0) <= minMax(1,0) &&
                otherMinMax(0,1) <= minMax(1,1) &&
                otherMinMax(0,2) <= minMax(1,2));
  return (first && second);
}
