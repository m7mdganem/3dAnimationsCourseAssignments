#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "igl/collapse_edge.h"
#include "AutoMorphingModel.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include "igl/per_vertex_normals.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/per_face_normals.h"
#include "igl/circulation.h"
using namespace cg3d;
using namespace std;
using namespace Eigen;
using namespace igl;
//done
void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    detect = true;
    velocity_x = 0;
    velocity_y = -0.001;
    auto m = [](Model* model, cg3d::Visitor* visitor)
    {
        return (model->GetMeshList())[0]->data.size() - 1;
    };
    camera = Camera::Create("camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();
    camera->Translate(10, Axis::Z);
    //declare the two objects and the 4 boxes
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    auto material2{ std::make_shared<Material>("textures/box0.bmp", program) };
    auto material3{ std::make_shared<Material>("textures/carbon.jpg", program) };
    material->AddTexture(0, "textures/bricks.jpg", 2);
    material2->AddTexture(0, "textures/box0.bmp", 2);
    material3->AddTexture(0, "textures/carbon.jpg", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/bunny.off") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/fertility.off") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cheburashka.off") };
    sphere1 = Model::Create("sphere", sphereMesh, material2);
    cyl = Model::Create("cyl", cylMesh, material2);
    cube = Model::Create("cube", cubeMesh, material2);
    auto Box1{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto Box2{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto Box3{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto Box4{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
    auto sphere_obj1{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto sphere_obj2{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    obj1 = Model::Create("sphere_obj1", sphere_obj1, material);
    obj2 = Model::Create("sphere_obj2", sphere_obj2, material);
    Big_box_obj1 = Model::Create("big_box1", Box1, material2);
    Big_box_obj2 = Model::Create("big_box2", Box2, material2);
    Small_box_obj1 = Model::Create("small_box1", Box3, material3);
    Small_box_obj2 = Model::Create("small_box2", Box4, material3);
    //make the faces transparent to make sure the small box can be seen (after the collision)
    Small_box_obj1->showFaces = true;
    Small_box_obj2->showFaces = true;
    Small_box_obj1->showWireframe = true;
    Small_box_obj2->showWireframe = true;
    Small_box_obj1->showFaces = false;
    Small_box_obj2->showFaces = false;
    Small_box_obj1->showWireframe = false;
    Small_box_obj2->showWireframe = false;
    Big_box_obj1->showFaces = false;
    Big_box_obj2->showFaces = false;
    Big_box_obj1->showWireframe = true;
    Big_box_obj2->showWireframe = true;
    obj1->showFaces = false;
    obj2->showFaces = false;
    obj1->showWireframe = false;
    obj2->showWireframe = false;
    final_obj1 = AutoMorphingModel::Create(*obj1, m);
    final_obj2 = AutoMorphingModel::Create(*obj2, m);
    root->AddChild(final_obj1);
    root->AddChild(final_obj2);
    final_obj1->AddChild(Big_box_obj1);
    final_obj1->Translate({ 0, 3, 0 });
    final_obj2->AddChild(Big_box_obj2);
    final_obj2->Translate({ 0, -3, 0 });
    //initlize the trees and update the boxes
    obj1_tree.init(final_obj1->GetMeshList()[0]->data[0].vertices, final_obj1->GetMeshList()[0]->data[0].faces);
    Build_BOX(obj1_tree.m_box, Big_box_obj1);
    obj2_tree.init(final_obj2->GetMeshList()[0]->data[0].vertices, final_obj2->GetMeshList()[0]->data[0].faces);
    Build_BOX(obj2_tree.m_box, Big_box_obj2);
}
//done
void BasicScene::Update(const Program& program, const Matrix4f& proj, const Matrix4f& view, const Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //if there is collision and we didn't reset it then add the boxes and reset the velocity 
    if (detect && isCollision(&obj1_tree, &obj2_tree))
    {
        final_obj1->AddChild(Small_box_obj2);
        final_obj2->AddChild(Small_box_obj1);
        Small_box_obj1->showFaces = true;
        Small_box_obj2->showFaces = true;
        Small_box_obj1->showWireframe = true;
        Small_box_obj2->showWireframe = true;
        velocity_x = 0;
        velocity_y = 0;
        detect = false;
    }
    final_obj1->Translate({ velocity_x, velocity_y, 0 });
    final_obj2->Translate(0, Axis::X);
}



//done
bool BasicScene::isCollision(AABB<MatrixXd, 3>* tree1, AABB<MatrixXd, 3>* tree2)
{
    //if one of them is null
    if (!(tree1 && tree2)) return false;
    //if there is collision between the boxes
    if (!CD_OBB(tree1->m_box, tree2->m_box)) return false;
    //if one of them is leaf
    if (tree1->is_leaf() && !tree2->is_leaf()) 
        return isCollision(tree1, tree2->m_right) || isCollision(tree1, tree2->m_left);
    if (!tree1->is_leaf() && tree2->is_leaf()) return isCollision(tree1->m_right, tree2) || isCollision(tree1->m_left, tree2);
    //if both of them are leafs 
    if (!tree1->is_leaf() && !tree2->is_leaf())
        return isCollision(tree1->m_right, tree2->m_right) || isCollision(tree1->m_left, tree2->m_right) || isCollision(tree1->m_right, tree2->m_left) || isCollision(tree1->m_left, tree2->m_left);
    //update the boxes
    Build_BOX(tree1->m_box, Small_box_obj1);
    Build_BOX(tree2->m_box, Small_box_obj2);
    return true;
}
//done
bool BasicScene::CD_OBB(AlignedBox<double, 3>& m_box1, AlignedBox<double, 3>& m_box2)

{

    //A
    A = final_obj1->GetRotation().cast<double>();
     A0 = A.col(0);
     A1 = A.col(1);
     A2 = A.col(2);
    //B
    B = final_obj2->GetRotation().cast<double>();
     B0 = B.col(0);
     B1 = B.col(1);
     B2 = B.col(2);
    //D
     D = (final_obj1->GetTransform().cast<double>() * (Vector4d(m_box2.center()[0], m_box2.center()[1], m_box2.center()[2], 1)) - final_obj2->GetTransform().cast<double>() * (Vector4d(m_box1.center()[0], m_box1.center()[1], m_box1.center()[2], 1))).head(3);
    //C
     C = A.transpose() * B;
     C00 = C.row(0)(0);
     C01 = C.row(0)(1);
     C02 = C.row(0)(2);
     C10 = C.row(1)(0);
     C11 = C.row(1)(1);
     C12 = C.row(1)(2);
     C20 = C.row(2)(0);
     C21 = C.row(2)(1);
     C22 = C.row(2)(2);
    //a
     a = m_box1.sizes() / 2;
     a0 = a(0);
     a1 = a(1);
     a2 = a(2);
    //b
     b = m_box2.sizes() / 2;
     b0 = b(0);
     b1 = b(1);
     b2 = b(2);
    
    //L=A0
    R0 = a0;
    R1 = b0 * abs(C00) + b1 * abs(C01) + b2 * abs(C02);
    R = abs(A0.transpose() * D);
    if (R0 + R1 < R) 
        return false;
    //L=A1
    R0 = a1;
    R1 = b0 * abs(C10) + b1 * abs(C11) + b2 * abs(C12);
    R = abs(A1.transpose() * D);
    if (R0 + R1 < R)
        return false;
    //L=A2
    R0 = a2;
    R1 = b0 * abs(C20) + b1 * abs(C21) + b2 * abs(C22);
    R = abs(A2.transpose() * D);
    if (R0 + R1 < R)
        return false;
    //L=B0
    R0 = a0 * abs(C00) + a1 * abs(C10) + a2 * abs(C20);
    R1 = b0;
    R = abs(B0.transpose() * D);
    if (R0 + R1 < R)
        return false;
    //L=B1
    R0 = a0 * abs(C01) + a1 * abs(C11) + a2 * abs(C21);
    R1 = b1;
    R = abs(B1.transpose() * D);
    if (R0 + R1 < R)
        return false;
    //L=B2
    R0 = a0 * abs(C02) + a1 * abs(C12) + a2 * abs(C22);
    R1 = b2;
    R = abs(B2.transpose() * D);
    if (R0 + R1 < R) 
        return false;
    //L=A0XB0
    R0 = a1 * abs(C20) + a2 * abs(C10);
    R1 = b1 * abs(C02) + b2 * abs(C01);
    R = C10 * A2.transpose() * D;
    R = abs(R - (C20 * A1.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //L=A0XB1
    R0 = a1 * abs(C21) + a2 * abs(C11);
    R1 = b0 * abs(C02) + b2 * abs(C00);
    R = C11 * A2.transpose() * D;
    R = abs(R - (C21 * A1.transpose() * D));
    if (R0 + R1 < R)
        return false;
    //L=A0XB2
    R0 = a1 * abs(C22) + a2 * abs(C12);
    R1 = b0 * abs(C01) + b1 * abs(C00);
    R = C12 * A2.transpose() * D;
    R = abs(R - (C22 * A1.transpose() * D));
    if (R0 + R1 < R)
        return false;
    //L=A1XB0
    R0 = a0 * abs(C20) + a2 * abs(C00);
    R1 = b1 * abs(C12) + b2 * abs(C11);
    R = C20 * A0.transpose() * D;
    R = abs(R - (C00 * A2.transpose() * D));
    if (R0 + R1 < R) return false;
    //L=A1XB1
    R0 = a0 * abs(C21) + a2 * abs(C01);
    R1 = b0 * abs(C12) + b2 * abs(C10);
    R = C21 * A0.transpose() * D;
    R = abs(R - (C01 * A2.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //L=A1XB2
    R0 = a0 * abs(C22) + a2 * abs(C02);
    R1 = b0 * abs(C11) + b1 * abs(C10);
    R = C22 * A0.transpose() * D;
    R = abs(R - (C02 * A2.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //L=A2XB0
    R0 = a0 * abs(C10) + a1 * abs(C00);
    R1 = b1 * abs(C22) + b2 * abs(C21);
    R = C00 * A1.transpose() * D;
    R = abs(R - (C10 * A0.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //L=A2XB1
    R0 = a0 * abs(C11) + a1 * abs(C01);
    R1 = b0 * abs(C22) + b2 * abs(C20);
    R = C01 * A1.transpose() * D;
    R = abs(R - (C11 * A0.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //L=A2XB1 its A2XB2 but in file its A2XB1
    R0 = a0 * abs(C12) + a1 * abs(C02);
    R1 = b0 * abs(C21) + b1 * abs(C20);
    R = C02 * A1.transpose() * D;
    R = abs(R - (C12 * A0.transpose() * D));
    if (R0 + R1 < R) 
        return false;
    //if all the 15 above conditions are satisfied (R0 + R1 >= R) then it's Collision
    return true;
}




//DONE
void BasicScene::Build_BOX(AlignedBox<double, 3>& m_box, shared_ptr<cg3d::Model> our_box)
{
    MatrixXd V;
    V.resize(8, 3);
    //adding the vertices 
    RowVector3d blc = m_box.corner(m_box.BottomLeftCeil);
    V.row(0) = blc;
    RowVector3d brf = m_box.corner(m_box.BottomRightFloor);
    V.row(1) = brf;
    RowVector3d brc = m_box.corner(m_box.BottomRightCeil);
    V.row(2) = brc;
    RowVector3d trc = m_box.corner(m_box.TopRightCeil);
    V.row(3) = trc;
    RowVector3d tlc = m_box.corner(m_box.TopLeftCeil);
    V.row(4) = tlc;
    RowVector3d tlf = m_box.corner(m_box.TopLeftFloor);
    V.row(5) = tlf;
    RowVector3d blf = m_box.corner(m_box.BottomLeftFloor);
    V.row(6) = blf;
    RowVector3d trf = m_box.corner(m_box.TopRightFloor);
    V.row(7) = trf; 
    MatrixXi F;
    //adding the faces 
    F.resize(12, 3);
    F << 6, 0, 4,
        6, 5, 4,
        5, 1, 7,
        5, 6, 1,
        3, 7, 1,
        1, 2, 3,
        4, 3, 2,
        4, 0, 2,
        4, 3, 5,
        5, 3, 7,
        0, 6, 2,
        6, 2, 1;
    MatrixXd Normal;
    per_vertex_normals(V, F, Normal);
    MatrixXd cordinat;
    cordinat = MatrixXd::Zero(V.rows(), 2);
    //update the box with the new vetices and faces 
    our_box->GetMeshList()[0]->data.push_back({ V,F,Normal,cordinat });
    our_box->SetMeshList(our_box->GetMeshList());
    our_box->meshIndex += 1;
}