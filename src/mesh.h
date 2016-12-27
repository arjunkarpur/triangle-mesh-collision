#ifndef __MESH_H__
#define __MESH_H__

#include "Eigen/Core"

// Struct for triangle mesh (vertices and faces/edges)
struct TriangleMesh {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

#endif
