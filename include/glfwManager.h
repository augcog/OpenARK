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
//#include "SegmentedMesh.h"

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

	void set_pos(int x, int y){
		glfwSetWindowPos(win_ptr,x,y);

	}

	virtual void keyboard_control()
	{
	    if(glfwGetKey(win_ptr, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	            glfwSetWindowShouldClose(win_ptr, GL_TRUE);
	}

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
// Moon: Cause 1 : class having FSVEO as members
class ARCameraWindow : public ObjectWindow{ 
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ARCameraWindow(std::string name, int resX, int resY, GLenum image_format, GLenum data_type, double px, double py, double cx, double cy, double near_cut, double far_cut):
	ObjectWindow(name,resX,resY),
	image_format_(image_format),
	data_type_(data_type),
	cube_num(0),
	near_cut_(near_cut),
	far_cut_(far_cut),
	px_(px),py_(py),cx_(cx),cy_(cy){
		proj_mat_=Eigen::Matrix4d::Zero();
		/*proj_mat_(0,0)=px/cx;
		proj_mat_(1,1)=py/cy;
		proj_mat_(2,2)=(near_cut+far_cut)/(near_cut-far_cut);
		proj_mat_(2,3)=2*far_cut*near_cut/(near_cut-far_cut);
		proj_mat_(3,2)=-1;*/
		proj_mat_(0,0)=2*px/resX;
		proj_mat_(1,1)=2*py/resY;
		proj_mat_(2,0)=2*cx/resX-1.0;
		proj_mat_(2,1)=2*cy/resY-1.0;
		proj_mat_(2,2)=(near_cut+far_cut)/(near_cut-far_cut);
		proj_mat_(2,3)=2*far_cut*near_cut/(near_cut-far_cut);
		proj_mat_(3,2)=-1;
		clicked_=false;
		glfwSetInputMode(win_ptr, GLFW_STICKY_MOUSE_BUTTONS, 1);
	}

	bool display() override;

	void set_camera(const Eigen::Affine3d& cam_extr){ // Moon: Cause 3 fine.
		cam_extr_=cam_extr;
	}


	void set_image(cv::Mat image_in);

	void keyboard_control() override;

	bool clicked();

	Eigen::Affine3d cam_extr();


private:
	Eigen::Affine3d cam_extr_; // Moon : a fixed-size vectorizable Eigen object
	Eigen::Matrix4d proj_mat_; // Moon : a fixed-size vectorizable Eigen object
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
	

	void keyboard_control()
	{
    float cameraSpeed = 0.05f; // adjust accordingly
    float zoomSpeed = 0.25f;
    if(glfwGetKey(win_ptr, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(win_ptr, GL_TRUE);
    if(glfwGetKey(win_ptr, GLFW_KEY_W) == GLFW_PRESS)
        eye = Eigen::Quaterniond(Eigen::AngleAxisd(-cameraSpeed, (gaze-eye).cross(Eigen::Vector3d::UnitY()).normalized()))*eye;//cameraSpeed * Eigen::Vector3d(0,1,0);
    if(glfwGetKey(win_ptr, GLFW_KEY_S) == GLFW_PRESS)
        eye = Eigen::Quaterniond(Eigen::AngleAxisd(cameraSpeed, (gaze-eye).cross(Eigen::Vector3d::UnitY()).normalized()))*eye;
    if(glfwGetKey(win_ptr, GLFW_KEY_A) == GLFW_PRESS)
    	eye = Eigen::Quaterniond(Eigen::AngleAxisd(cameraSpeed, Eigen::Vector3d::UnitY()))*eye;
    if(glfwGetKey(win_ptr, GLFW_KEY_D) == GLFW_PRESS)
    	eye = Eigen::Quaterniond(Eigen::AngleAxisd(-cameraSpeed, Eigen::Vector3d::UnitY()))*eye;
    if(glfwGetKey(win_ptr, GLFW_KEY_Z) == GLFW_PRESS)
        eye += zoomSpeed * (gaze-eye).normalized();
    if(glfwGetKey(win_ptr, GLFW_KEY_X) == GLFW_PRESS)
        eye -= zoomSpeed * (gaze-eye).normalized();
	}
};

class ImageWindow : public Window{
public:
	cv::Mat current_image;
	GLuint texture;
	GLenum image_format_;
	GLenum data_type_;

	//Image format examples: GL_RGB, GLBGR, GL_LUMINANCE
	//Data type examples GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, GL_FLOAT
	ImageWindow(std::string name, int resX, int resY, GLenum image_format, GLenum data_type):
	Window(name,resX,resY),
	image_format_(image_format),
	data_type_(data_type){
		glGenTextures(1,&texture);
	}


	void set_image(cv::Mat image_in);

	bool display();

};

class MeshWindow : public ObjectWindow { // Moon : Cause 1 : class having FSVEO as members
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	MeshWindow(std::string name, int resX, int resY) :
		ObjectWindow(name, resX, resY) {};
	//bool display();
	void set_camera(const Eigen::Affine3d& t) { // Moon : Cause 3. This might be wrong. Const + & or & alone should be added
		transform = t;
	}
        void keyboard_control()
	{
		if (glfwGetKey(win_ptr, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(win_ptr, GL_TRUE);
		if (glfwGetMouseButton(win_ptr, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
			clicked_ = true;
		}
	}

        bool clicked() {
		bool clicked = clicked_;
		clicked_ = false;
		return clicked;
	}
private:
	Eigen::Affine3d transform; // Moon : a fixed-size vectorizable eigen object
	bool clicked_ = false;
};
// Moon: Cause 1 : class having FSVEO as members 
class Object{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::map<std::string,ObjectWindow*> windows;
	std::string name_;
	Object(std::string name);

	virtual ~Object();

	void del();

	void display();

	void set_transform(const Eigen::Affine3d& t); // Moon : Cause 3. This might be wrong. Const + & or & might be needed.

	void translate(Eigen::Translation3d t);

	void rotate(const Eigen::Quaterniond& q); // Moon : Cause 3. This might be wrong. Const + & or & might be needed.

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

	Cube(std::string name, float width, float height, float length) : 
	Object(name), hWidth(width/2), hHeight(height/2), hLength(length/2){

	}
};

class Grid : public Object {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	float size, step;
	void draw_obj();

	Grid(std::string name, float size, float step) 
	: Object(name),
	size(size),
	step(step){

	}
};

class Axis : public Object {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	float size;
	void draw_obj();

	Axis(std::string name, float size) 
	: Object(name),
	size(size){

	}

};

class Path : public Object {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::vector<Eigen::Vector3d> nodes;
	Eigen::Vector3d color;
	void draw_obj();

	Path(std::string name)
	: Object(name),
	nodes(){
	}

	Path(std::string name, Eigen::Vector3d color)
	: Object(name),
	nodes(),
	color(color){
	}

	Path(std::string name, const std::vector<Eigen::Vector3d>& nodes) // Moon: Rule 3 fine.
	: Object(name),
	nodes(nodes){
	}

	void add_node(Eigen::Vector3d node){ 
		nodes.push_back(node);
	}

	void clear(){
		nodes.clear();
	}

	void set_color(Eigen::Vector3d obj_color){
		color = obj_color;
	}

	void set_color(float x, float y, float z){
		color = Eigen::Vector3d(x,y,z);
	}
};

// class Mesh : public Object {
// public:

// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// 	std::vector<std::vector<Eigen::Vector3d>> mesh_vertices;
//     std::vector<std::vector<Eigen::Vector3d>> mesh_colors;
//     std::vector<std::vector<Eigen::Vector3i>> mesh_triangles;
//     std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mesh_transforms; // Moon : Cause 2.a
//     std::vector<int> mesh_enabled;

// 	// //threadsafe?
// 	// int current_active_map;
// 	// int archive_index;
// 	// std::set<int> enabled_meshes;

// 	ark::SegmentedMesh * mesh_;

// 	void draw_obj();

// 	Mesh(std::string name, ark::SegmentedMesh * mesh)
// 	: Object(name) {
// 		mesh_ = mesh;
// 	}

// 	void update_meshes() {
// 		std::lock_guard<std::mutex> guard(meshLock_);

// 		mesh_->Render(mesh_vertices, mesh_colors, mesh_triangles, mesh_transforms, mesh_enabled); // Moon : Render is deprecated in the Master branch of OpenARK. This line might be wrong.

// 	}


// protected:
// 	std::mutex meshLock_;

// };


} //MyGUI