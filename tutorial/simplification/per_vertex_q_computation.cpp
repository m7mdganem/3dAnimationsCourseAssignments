#include "per_vertex_q_computation.h"

void per_vertex_q_computation(
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    std::vector <Eigen::Matrix4d>& VQ)
{
    VQ.resize(V.rows());
    std::vector<std::vector<int>> VF;
    std::vector<std::vector<int>> VFi;
    Eigen::MatrixXd FN;

    igl::vertex_triangle_adjacency(V.rows(), F, VF, VFi);
    igl::per_face_normals(V, F, FN);

    // Calculate Q for each vertex
    for (int vertex_index = 0; vertex_index < V.rows(); vertex_index++)
    {
        VQ[vertex_index] = Eigen::Matrix4d::Zero();
        int number_of_faces = VF[vertex_index].size();

        // Iterate over faces
        for (int face_index = 0; face_index < number_of_faces; face_index++)
        {
            Eigen::Vector3d normal = FN.row(VF[vertex_index][face_index]).normalized();
            double a = normal[0];
            double b = normal[1];
            double c = normal[2];
            double d = V.row(vertex_index) * normal;
            d *= -1;

            VQ[vertex_index].row(0) += Eigen::Vector4d(a * a, a * b, a * c, a * d);
            VQ[vertex_index].row(1) += Eigen::Vector4d(a * b, b * b, b * c, b * d);
            VQ[vertex_index].row(2) += Eigen::Vector4d(a * c, b * c, c * c, c * d);
            VQ[vertex_index].row(3) += Eigen::Vector4d(a * d, b * d, c * d, d * d);
        }
    }
}