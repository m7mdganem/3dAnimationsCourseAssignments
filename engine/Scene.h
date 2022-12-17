#pragma once
#include "MeshGL.h"
#include "Movable.h"
#include "Material.h"
#include "Camera.h"
#include "Model.h"
#include "Viewport.h"
#include "Display.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include <string>
#include <cstdint>
#include "glfw/Viewer.h"

namespace cg3d
{

class Display;

class Scene : public Movable, public igl::opengl::glfw::Viewer
{
protected:
    Scene(std::string name, Display* display);

public:
    ~Scene() override = default;
    void Accept(Visitor* visitor) override { visitor->Visit(this); };

    virtual void Init(Visitor* visitor);
    virtual void Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model);
    float velocity_x, velocity_y;
    bool detect;
    std::shared_ptr<Model> pickedModel;
    std::shared_ptr<Camera> camera;
    std::shared_ptr<cg3d::Model> obj1, obj2, Big_box_obj1, Small_box_obj1, Big_box_obj2, Small_box_obj2, final_obj1, final_obj2;
    virtual void MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]);
    virtual void ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]);
    virtual void CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState);
    virtual void KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods);
    virtual void CharCallback(Viewport* viewport, int x, int y, unsigned int codepoint);
    virtual void ViewportSizeCallback(Viewport* viewport);
    virtual void AddViewportCallback(Viewport* viewport) {};
    

protected:
    Renderer* renderer; // required for picking
    int xAtPress = -1, yAtPress = -1;
    float pickedModelDepth = 0;
    Eigen::Affine3f pickedToutAtPress, cameraToutAtPress;
};

} // namespace cg3d
