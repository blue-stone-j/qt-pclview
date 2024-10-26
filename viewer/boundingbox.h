#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "eigen3/Eigen/Dense"
#include <queue>

struct ObjectBox
{
  // these three are for displaying box
  Eigen::Vector3f trans;
  Eigen::Quaternionf rotation;
  Eigen::Vector3f size; // width, height, depth,
};

#endif // BOUNDINGBOX_H
