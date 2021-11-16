#pragma once
#ifdef _WIN32
    #include <Windows.h>
    #undef ERROR
    #include <gl/GLU.h>
#else
    //#include <GL/glew.h>
    #include <GL/glu.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <atomic>
#include <mutex>
#include <set>
#include "util/SegmentedMesh.h"

#include <opencv2/core/core.hpp>


namespace MyGUI{



class Window;
class Object;

class Manager {
public:
    static std::map<std::string,Window*> windows;

    //This function simply references GLFW init
    static int (&init)(); 

    static void (&terminate)(); 

    static bool running();

    static void update();

};//Manager

class Window{
public:
    GLFWwindow* win_ptr;
    std::string name_;

    Window(std::string name, int resX, int resY);

    virtual ~Window();

    virtual bool display();

    void add_control_func(GLFWkeyfun controls);

    void set_pos(int x, int y);

    virtual void keyboard_control();

};

class ObjectWindow : public Window{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::map<std::string,Object*> objects;

    Eigen::Vector3d eye;
    Eigen::Vector3d gaze;
    std::string msg_;

    ObjectWindow(std::string name, int resX, int resY);

    virtual ~ObjectWindow();

    virtual bool display();

    void add_object(Object* obj);

    //void set_camera(const Eigen::Vector3d& eye, const Eigen::Vector3d& gaze);

protected:
    std::mutex displayLock_;


};//Window

class ARCameraWindow : public ObjectWindow{ 
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ARCameraWindow(std::string name, int resX, int resY, GLenum image_format, GLenum data_type, double px, double py, double cx, double cy, double near_cut, double far_cut);

    bool display() override;

    void set_camera(const Eigen::Affine3d& cam_extr);


    void set_image(cv::Mat image_in);

    void keyboard_control() override;

    bool clicked();

    Eigen::Affine3d cam_extr();


private:
    Eigen::Affine3d cam_extr_; 
    Eigen::Matrix4d proj_mat_; 
    cv::Mat current_image;
    GLuint texture;
    GLenum image_format_;
    GLenum data_type_;
    int cube_num;
    double near_cut_;
    double far_cut_;
    double px_,py_,cx_,cy_;
    std::atomic<bool> clicked_;
};


class CameraWindow : public ObjectWindow{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraWindow(std::string name, int resX, int resY):
    ObjectWindow(name,resX,resY){}
    

    void keyboard_control();
};

class ImageWindow : public Window{
public:
    cv::Mat current_image;
    GLuint texture;
    GLenum image_format_;
    GLenum data_type_;

    //Image format examples: GL_RGB, GLBGR, GL_LUMINANCE
    //Data type examples GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, GL_FLOAT
    ImageWindow(std::string name, int resX, int resY, GLenum image_format, GLenum data_type);


    void set_image(cv::Mat image_in);

    bool display();

};

class MeshWindow : public ObjectWindow { 
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MeshWindow(std::string name, int resX, int resY) :
        ObjectWindow(name, resX, resY) {};
    //bool display();
    void set_camera(const Eigen::Affine3d& t);
    void keyboard_control();

    bool clicked();
private:
    Eigen::Affine3d transform; 
    bool clicked_ = false;
};

class Object{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::map<std::string,ObjectWindow*> windows;
    std::string name_;
    Object(std::string name);

    virtual ~Object();

    void del();

    void display();

    void set_transform(const Eigen::Affine3d& t);

    void translate(Eigen::Translation3d t);

    void rotate(const Eigen::Quaterniond& q); 

    void hide();

    void show();

    virtual void draw_obj();

protected:
    std::mutex displayLock_;
    Eigen::Affine3d pose; // Eigen : Fixed-size vectorizable Eigen object
    bool draw;
    
};//Object

class Cube : public Object {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float hWidth, hHeight, hLength;
    void draw_obj();

    Cube(std::string name, float width, float height, float length);
};

class Grid : public Object {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float size, step;
    void draw_obj();

    Grid(std::string name, float size, float step);
};

class Axis : public Object {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float size;
    void draw_obj();

    Axis(std::string name, float size);

};

class Path : public Object {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> nodes;
    Eigen::Vector3d color;
    void draw_obj();

    Path(std::string name);

    Path(std::string name, Eigen::Vector3d color);

    Path(std::string name, const std::vector<Eigen::Vector3d>& nodes);

    void add_node(Eigen::Vector3d node);

    void clear();

    void set_color(Eigen::Vector3d obj_color);

    void set_color(float x, float y, float z);
};

class Mesh : public Object {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>> mesh_vertices;
    std::shared_ptr<std::vector<std::vector<Eigen::Vector3d>>> mesh_colors;
    std::shared_ptr<std::vector<std::vector<Eigen::Vector3i>>> mesh_triangles;
    std::shared_ptr<std::vector<Eigen::Matrix4d>> mesh_transforms;
    std::shared_ptr<std::vector<int>> mesh_enabled;

    // //threadsafe?
    // int current_active_map;
    // int archive_index;
    // std::set<int> enabled_meshes;

    ark::SegmentedMesh * mesh_;

    void draw_obj();

    Mesh(std::string name, ark::SegmentedMesh * mesh);

protected:
    std::mutex meshLock_;

};


} //MyGUI
