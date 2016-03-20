/*
 * GLRenderer.hpp
 *
 *  Created on: Jan 10, 2016
 *      Author: neeravbm
 */
#pragma once

#ifndef GLRENDERER_HPP_
#define GLRENDERER_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <Shader.hpp>
#include <glm/glm.hpp>
#include <Camera.hpp>
#include <Utils.hpp>

class GLRenderer {

public:
	GLRenderer();
	GLRenderer(int width, int height);
	GLRenderer(int width, int height, glm::vec3 bgColor);
	void addPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
			glm::vec3 color = glm::vec3(-1, -1, -1));
	void addAxes();
	void renderPlane();
	void render();
	enum Primitive {
		points, triangles, lines
	};
	enum Type {
		Data, Normal, Axes
	};

private:
	struct RenderStruct {
		GLuint VAO;
		Primitive primitive;
		GLuint num;
		Shader shader;
		bool instanced;
		Type type;
		GLuint VBO; // Pointer stored so that they can be cleared later.
		GLuint EBO; // Pointer stored so that they can be cleared later.
	};
	std::vector<RenderStruct> renderStructs;
	void clearBuffers();
	static bool keys[1024];
	static void key_callback(GLFWwindow* window, int key, int scancode,
			int action, int mode);
	static void scroll_callback(GLFWwindow* window, double xoffset,
			double yoffset);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void mouse_button_callback(GLFWwindow* window, int button,
			int action, int mods);
	//static void keyboardUp(unsigned char key, int x, int y);
	GLFWwindow* window;
	glm::vec3 bgColor;
	static Camera camera;
	static GLfloat lastX, lastY, firstX, firstY;
	static bool firstMouse, leftMouseButtonPressed, refreshWindow, selectPoints, showNormals;
	long int lastFrame;
	long int getNanoCount();
	long int getNanoSpan(long int nTimeStart, long int nTimeEnd);
	void doMovement(GLfloat deltaTime);
	int width, height;
	int pointSize;
	static std::shared_ptr<spdlog::logger> logger;
};

#endif /* GLRENDERER_HPP_ */
