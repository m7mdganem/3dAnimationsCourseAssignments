#pragma once

#include "Scene.h"
#include "igl/AABB.h"

#include <utility>
using namespace Eigen;
using namespace igl;

class BasicScene : public cg3d::Scene
{
public:
    void Build_BOX(Eigen::AlignedBox<double, 3>& m_box, std::shared_ptr<cg3d::Model> our_box);
    bool isCollision(igl::AABB<Eigen::MatrixXd, 3>* object_tree1, igl::AABB<Eigen::MatrixXd, 3>* object_tree2);
    bool CD_OBB(Eigen::AlignedBox<double, 3>& aligned_box1, Eigen::AlignedBox<double, 3>& aligned_box2);
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cube, cyl, autoCube, sphere1, autoSphere, autoCyl;
    igl::AABB<Eigen::MatrixXd, 3> obj1_tree, obj2_tree;
    Matrix3d A, B, C;
    Vector3d A0, A1, A2, B0, B1, B2, D, a, b;
    double R0, R1, R, a0, a1, a2, b0, b1, b2, C00, C01, C02, C10, C11, C12, C20, C21, C22;
};
