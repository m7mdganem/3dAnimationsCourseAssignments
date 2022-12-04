#pragma once

#include <Eigen/Core>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>

void per_vertex_q_computation(
    Eigen::MatrixXd& V                  , /* Verticies */
    Eigen::MatrixXi& F                  , /* Faces */
    std::vector <Eigen::Matrix4d>& VQ     /* Vreticies to matricies */
);