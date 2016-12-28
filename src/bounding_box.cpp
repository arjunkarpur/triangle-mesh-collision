#include "bounding_box.h"

BoundingBox::BoundingBox(Eigen::MatrixXd points) {
  Eigen::RowVector3d min = points.colwise().minCoeff();
  Eigen::RowVector3d max = points.colwise().maxCoeff();
  minMax = Eigen::MatrixXd(2,3);
  minMax << min, max;
}

Eigen::MatrixXd BoundingBox::getMinMax() {
  return minMax;
}

Eigen::MatrixXd BoundingBox::getCorners() {
  //TODO
  return Eigen::MatrixXd(1,1);
}

bool BoundingBox::intersectsWith(BoundingBox other) {
  //TODO
  return false;
}
