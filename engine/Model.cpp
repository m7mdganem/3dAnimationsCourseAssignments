#include "Model.h"
#include "Visitor.h"
#include "ViewerData.h"
#include "Movable.h"
#include "ObjLoader.h"
#include <filesystem>
#include <utility>

namespace cg3d
{

namespace fs = std::filesystem;

Model::Model(const std::string& file, std::shared_ptr<Material> material)
        : Model{fs::path(file).filename().stem().generic_string(), file, std::move(material)} {}

Model::Model(std::string name, const std::string& file, std::shared_ptr<Material> material)
        : Model{*ObjLoader::ModelFromObj(std::move(name), file, std::move(material))} {}

Model::Model(std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> material)
        : Model{mesh->name + "_model", std::move(mesh), std::move(material)} {};

Model::Model(std::string name, std::shared_ptr<Mesh> mesh, std::shared_ptr<Material> material)
        : Model{std::move(name), std::vector<std::shared_ptr<Mesh>>{{std::move(mesh)}}, std::move(material)} {}

Model::Model(std::string name, std::vector<std::shared_ptr<Mesh>> meshList, std::shared_ptr<Material> material)
        : Movable(std::move(name)), material(std::move(material))
{
    SetMeshList(std::move(meshList));
}

std::vector<igl::opengl::ViewerData> Model::CreateViewerData(const std::shared_ptr<Mesh>& mesh)
{
    std::vector<igl::opengl::ViewerData> dataList;

    for (auto& meshData: mesh->data) {
        igl::opengl::ViewerData viewerData;
        viewerData.set_mesh(meshData.vertices, meshData.faces);
        viewerData.set_uv(meshData.textureCoords);
        viewerData.set_normals(meshData.vertexNormals);
        viewerData.line_width = 1.0f;
        viewerData.uniform_colors(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0)); // todo: implement colors
        viewerData.compute_normals(); // todo: implement (this overwrites both face and vertex normals even if either is already present)
        if (viewerData.V_uv.rows() == 0)
            viewerData.grid_texture();
        viewerData.is_visible = 1;
        viewerData.show_overlay = 0;
        dataList.emplace_back(std::move(viewerData));
    }

    return dataList;
}

void Model::UpdateDataAndBindMesh(igl::opengl::ViewerData& viewerData, const Program& program)
{
    viewerData.dirty = igl::opengl::MeshGL::DIRTY_NONE;
    viewerData.updateGL(viewerData, viewerData.invert_normals, viewerData.meshgl);
    viewerData.meshgl.shader_mesh = program.GetHandle();
    viewerData.meshgl.bind_mesh();
}

void Model::UpdateDataAndDrawMeshes(const Program& program, bool _showFaces, bool bindTextures)
{
    for (auto& viewerDataList: viewerDataListPerMesh) {
        auto& viewerData = viewerDataList[std::min(meshIndex, int(viewerDataList.size() - 1))];
        UpdateDataAndBindMesh(viewerData, program);
        if (bindTextures) material->BindTextures();
        viewerData.meshgl.draw_mesh(_showFaces);
    }
}

void Model::SetMeshList(std::vector<std::shared_ptr<Mesh>> _meshList)
{
    meshList = std::move(_meshList);
    viewerDataListPerMesh.clear();
    for (auto& mesh: meshList)
        viewerDataListPerMesh.emplace_back(CreateViewerData(mesh));
}

void Model::Simplify(bool use_igl_collapse_edge)
{
    bool something_collapsed = false;
    auto mesh_list = this->GetMeshList();
    int local_max_mesh_data_size = 0;
    for (int index = 0; index < meshList.size(); index++)
    {
        std::shared_ptr<Mesh> mesh = this->GetMesh(index);
        int currentDataIndex = mesh->data.size() - 1;
        auto& extended_data = mesh->extended_data[currentDataIndex];
        int number_of_faces_to_delete = 2 * std::ceil(0.1 * extended_data.edges_count);
        something_collapsed = mesh->Simplify(number_of_faces_to_delete, use_igl_collapse_edge);
        local_max_mesh_data_size = std::max(local_max_mesh_data_size, (int)mesh->data.size() - 1);
    }
    this->max_mesh_data_size = local_max_mesh_data_size;
    this->SetMeshList(mesh_list);
    if (something_collapsed)
    {
        this->meshIndex++;
    }
}

} // namespace cg3d
