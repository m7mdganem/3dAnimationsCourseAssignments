#pragma once

#include <Eigen/Core>
#include <iostream>
#include <utility>
#include <vector>
#include <igl/decimate.h> // for the min_heap
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>
#include "igl/per_vertex_normals.h"
#include "../tutorial/simplification/edges_cost_computation.h"
#include "../tutorial/simplification/per_vertex_q_computation.h"
#include "../tutorial/simplification/collapse_edge_improved.h"

namespace cg3d
{

struct MeshData
{
    const Eigen::MatrixXd vertices; // Vertices of the mesh (#V x 3)
    const Eigen::MatrixXi faces; // Faces of the mesh (#F x 3)
    const Eigen::MatrixXd vertexNormals; // One normal per vertex
    const Eigen::MatrixXd textureCoords; // UV vertices
};

struct MeshExtendedData
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXi E;
    Eigen::MatrixXi EF;
    Eigen::MatrixXi EI;
    Eigen::MatrixXd C;
    Eigen::VectorXi EQ;
    Eigen::VectorXi EMAP;
    igl::min_heap<std::tuple<double, int, int>> Q;
    int edges_count;

    std::vector <Eigen::Matrix4d> VQ;
    std::vector<double> EC;
};

class Mesh
{
public:
    std::string name;

    std::vector<MeshData> data;
    std::vector<MeshExtendedData> extended_data;

    Mesh(std::string name, Eigen::MatrixXd vertices, Eigen::MatrixXi faces, Eigen::MatrixXd vertexNormals, Eigen::MatrixXd textureCoords);
    Mesh(std::string name, std::vector<MeshData> data) : name(std::move(name)), data(std::move(data)), extended_data(std::move(extract_mesh_extended_data_from_mesh_data_improved(this->data))) {};
    Mesh(const Mesh& mesh) = default;

    std::vector<MeshExtendedData> extract_mesh_extended_data_from_mesh_data(std::vector<MeshData> data);
    MeshExtendedData extract_mesh_extended_data(Eigen::MatrixXd vertices, Eigen::MatrixXi faces);

    std::vector<MeshExtendedData> extract_mesh_extended_data_from_mesh_data_improved(std::vector<MeshData> data);
    MeshExtendedData extract_mesh_extended_data_improved(Eigen::MatrixXd vertices, Eigen::MatrixXi faces);

    bool Simplify(int number_of_faces_to_delete, bool use_igl_collapse_edge);

    static const std::shared_ptr<Mesh>& Plane();
    static const std::shared_ptr<Mesh>& Cube();
    static const std::shared_ptr<Mesh>& Tetrahedron();
    static const std::shared_ptr<Mesh>& Octahedron();
    static const std::shared_ptr<Mesh>& Cylinder();
};

} // namespace cg3d
