/*
 * GLRenderer.cpp
 *
 *  Created on: Jan 10, 2016
 *      Author: neeravbm
 */

#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include <GLRenderer.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <pcl/filters/filter.h>

Camera GLRenderer::camera = Camera(glm::vec3(0.0f, 0.0f, 3.0f));
bool GLRenderer::firstMouse = true, GLRenderer::leftMouseButtonPressed = false,
		GLRenderer::refreshWindow = true, GLRenderer::selectPoints = false,
		GLRenderer::showNormals = false;
bool GLRenderer::keys[] = { };
GLfloat GLRenderer::lastX = 0, GLRenderer::lastY = 0, GLRenderer::firstX = 0,
		GLRenderer::firstY = 0;
std::shared_ptr<spdlog::logger> GLRenderer::logger = spdlog::get("console");

void GLRenderer::key_callback(GLFWwindow* window, int key, int scancode,
		int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
		refreshWindow = true;
	}
	if (key >= 0 && key < 1024) {
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
		refreshWindow = true;
	}
}

void GLRenderer::mouse_button_callback(GLFWwindow* window, int button,
		int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			leftMouseButtonPressed = true;
		} else if (action == GLFW_RELEASE) {
			leftMouseButtonPressed = false;
			firstMouse = true;
		}
		refreshWindow = true;
	}
}

void GLRenderer::mouse_callback(GLFWwindow* window, double xpos, double ypos) {
	if (!leftMouseButtonPressed) {
		return;
	}

	if (firstMouse) {
		lastX = xpos;
		lastY = ypos;
		firstX = xpos;
		firstY = ypos;
		firstMouse = false;
		return;
	}

	if (!selectPoints) {
		logger->debug() << "Old mouse position: " << glm::vec2(lastX, lastY);
		logger->debug() << "New mouse position: " << glm::vec2(xpos, ypos);
		GLfloat xoffset = xpos - lastX;
		GLfloat yoffset = ypos - lastY; // Reversed since y-coordinates go from bottom to left

		lastX = xpos;
		lastY = ypos;

		camera.processMouseMovement(xoffset, yoffset);
	}
	refreshWindow = true;
}

void GLRenderer::scroll_callback(GLFWwindow* window, double xoffset,
		double yoffset) {
	camera.processMouseScroll(yoffset);
	refreshWindow = true;
}

GLRenderer::GLRenderer(int width, int height, glm::vec3 bgColor) {
	this->width = width;
	this->height = height;
	this->bgColor = bgColor;
	this->lastFrame = 0;
	this->pointSize = 1;

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	//glEnable(GL_PROGRAM_POINT_SIZE);

	GLFWwindow* window = glfwCreateWindow(width, height, "LearnOpenGL", nullptr,
	nullptr);
	if (window == nullptr) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		//return -1;
	}

	glfwMakeContextCurrent(window);

	if (gl3wInit()) {
		std::cerr << "Failed to initialize OpenGL" << std::endl;
		//return -1;
	}
	if (!gl3wIsSupported(3, 2)) {
		std::cerr << "OpenGL 3.2 not supported" << std::endl;
		//return -1;
	}

	glViewport(0, 0, width, height);

	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	// Options
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	glEnable(GL_DEPTH_TEST);

	this->window = window;
	//return 0;
}

GLRenderer::GLRenderer(int width, int height) :
		GLRenderer(width, height, glm::vec3(0.0f, 0.0f, 0.0f)) {
}

GLRenderer::GLRenderer() :
		GLRenderer(800, 600) {
}

void GLRenderer::render() {
	while (!glfwWindowShouldClose(window)) {
		long int currentFrame = getNanoCount();
		long int deltaTime = getNanoSpan(lastFrame, currentFrame);
		long int lastFrameTime = lastFrame;
		lastFrame = currentFrame;

		glfwPollEvents();
		if (refreshWindow) {
			logger->trace() << "Last frame: " << lastFrameTime;
			logger->trace() << "Current frame: " << currentFrame;
			logger->debug() << "Time between frames: "
					<< (GLfloat) deltaTime / 1000000.0f << " ms";
			this->doMovement((GLfloat) deltaTime / 1000000.0f);

			glClearColor(this->bgColor.r, this->bgColor.g, this->bgColor.b,
					1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glm::mat4 view, projection;
			view = camera.getViewMatrix();
			projection = glm::perspective(camera.Zoom,
					(float) this->width / (float) this->height, 0.1f, 1000.0f);

			glm::mat4 model = glm::mat4();

			for (auto &renderStruct : this->renderStructs) {

				if (!showNormals && renderStruct.type == GLRenderer::Normal) {
					continue;
				}

				renderStruct.shader.use();

				renderStruct.shader.setParamMatrix4fv("model", model);
				renderStruct.shader.setParamMatrix4fv("view", view);
				renderStruct.shader.setParamMatrix4fv("projection", projection);

				glBindVertexArray(renderStruct.VAO);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderStruct.EBO);

				switch (renderStruct.primitive) {
				case GLRenderer::points:
					if (renderStruct.instanced) {
						glPointSize(this->pointSize);
						glDrawElementsInstanced(GL_POINTS, 1, GL_UNSIGNED_INT,
								0, renderStruct.num);
					} else {
						glDrawElements(GL_POINTS, renderStruct.num,
						GL_UNSIGNED_INT, 0);
					}
					break;
				case GLRenderer::triangles:
					if (renderStruct.instanced) {
						glDrawArraysInstanced(GL_TRIANGLES, 0, 1,
								renderStruct.num);
					} else {
						glDrawArrays(GL_TRIANGLES, 0, renderStruct.num);
					}
					break;
				case GLRenderer::lines:
					if (renderStruct.instanced) {
						glDrawElementsInstanced(GL_LINES, renderStruct.num,
						GL_UNSIGNED_INT, 0, 1);
					} else {
						glDrawElements(GL_LINES, renderStruct.num,
						GL_UNSIGNED_INT, 0);
					}
				}

				renderStruct.shader.unUse();
			}

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			glBindVertexArray(0);

			glfwSwapBuffers(window);
			refreshWindow = false;
		}
	}

	this->clearBuffers();

	glfwTerminate();
}

void GLRenderer::clearBuffers() {
	for (auto &renderStruct : this->renderStructs) {
		glDeleteVertexArrays(1, &renderStruct.VAO);
		glDeleteBuffers(1, &renderStruct.VBO);
		glDeleteBuffers(1, &renderStruct.EBO);
		/*for (auto VBO : renderStruct.VBOs) {
		 glDeleteBuffers(1, &VBO);
		 }
		 for (auto EBO : renderStruct.EBOs) {
		 glDeleteBuffers(1, &EBO);
		 }*/
		//glDeleteBuffers(1, &EBO);
	}
}

void GLRenderer::addAxes() {
	Shader axesShader = Shader("../shaders/vertexshader_axes.glsl",
			"../shaders/fragmentshader_pc.glsl");
	axesShader.use();

	// Draw x,y and z axes at the origin.
	std::vector<glm::vec3> vertices;

	// Point (0, 0, 0)
	vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	// Color red
	vertices.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	// Point (1, 0, 0)
	vertices.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	// Color red
	vertices.push_back(glm::vec3(1.0f, 0.0f, 0.0f));

	// Point (0, 0, 0)
	vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	// Color green
	vertices.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	// Point (0, 1, 0)
	vertices.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	// Color green
	vertices.push_back(glm::vec3(0.0f, 1.0f, 0.0f));

	// Point (0, 0, 0)
	vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	// Color blue
	vertices.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	// Point (0, 0, 1)
	vertices.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	// Color blue
	vertices.push_back(glm::vec3(0.0f, 0.0f, 1.0f));

	GLuint indices[] = { 0, 1, 2, 3, 4, 5 };

	GLuint instanceVBO;
	glGenBuffers(1, &instanceVBO);
	glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	glBufferData(GL_ARRAY_BUFFER, //sizeof(vertices), &vertices[0],
			3 * sizeof(GLfloat) * vertices.size(), &vertices[0],
			GL_STATIC_DRAW);
	//glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLuint VAO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	GLuint EBO;
	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
	GL_STATIC_DRAW);

	// Also set instance data
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
			(GLvoid*) 0);
	//glVertexAttribDivisor(0, 1); // Tell OpenGL this is an instanced vertex attribute.

	// Bind color
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat),
			(GLvoid*) (3 * sizeof(GLfloat)));
	//glVertexAttribDivisor(1, 1); // Tell OpenGL this is an instanced vertex attribute.

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	axesShader.unUse();

	RenderStruct renderStruct = { };
	renderStruct.VAO = VAO;
	renderStruct.num = 6;
	renderStruct.primitive = GLRenderer::lines;
	renderStruct.shader = axesShader;
	renderStruct.instanced = false;
	renderStruct.VBO = instanceVBO;
	renderStruct.EBO = EBO;
	renderStruct.type = GLRenderer::Axes;
	this->renderStructs.push_back(renderStruct);
}

void GLRenderer::addPointCloud(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, glm::vec3 color) {
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	// Render the point cloud.
	Shader pcShader = Shader("../shaders/vertexshader_pc.glsl",
			"../shaders/fragmentshader_pc.glsl");
	pcShader.use();

	const int pc_size = cloud->points.size();
	std::vector<GLfloat> vertices;
	for (GLuint i = 0; i < 3 * pc_size; i += 3) {
		vertices.push_back(cloud->points[i / 3].x);
		vertices.push_back(cloud->points[i / 3].y);
		vertices.push_back(cloud->points[i / 3].z);
	}

	// Add normals
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normalCloud(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<int> normalIndices;
	pcl::removeNaNNormalsFromPointCloud(*cloud, *normalCloud, normalIndices);
	const int normal_pc_size = normalCloud->points.size();
	for (int i = 0; i < 3 * normal_pc_size; i += 3) {
		vertices.push_back(
				normalCloud->points[i / 3].x
						+ 0.01 * normalCloud->points[i / 3].normal_x);
		vertices.push_back(
				normalCloud->points[i / 3].y
						+ 0.01 * normalCloud->points[i / 3].normal_y);
		vertices.push_back(
				normalCloud->points[i / 3].z
						+ 0.01 * normalCloud->points[i / 3].normal_z);
	}

	// Add vertex color
	for (GLuint i = 0; i < 3 * pc_size; i += 3) {
		vertices.push_back(
				color.x < 0.0f ?
						(GLfloat) cloud->points[i / 3].r / 255 : color.x);
		vertices.push_back(
				color.y < 0.0f ?
						(GLfloat) cloud->points[i / 3].g / 255 : color.y);
		vertices.push_back(
				color.z < 0.0f ?
						(GLfloat) cloud->points[i / 3].b / 255 : color.z);
	}

	// Add normal color
	vertices.push_back(0.0f);
	vertices.push_back(0.0f);
	vertices.push_back(1.0f);

	GLuint dataVBO;
	glGenBuffers(1, &dataVBO);
	glBindBuffer(GL_ARRAY_BUFFER, dataVBO);
	glBufferData(GL_ARRAY_BUFFER, //sizeof(vertices), &vertices[0],
			sizeof(GLfloat) * vertices.size(), &vertices[0],
			GL_STATIC_DRAW);

	GLuint VAO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	std::vector<GLuint> indices;
	for (GLuint i = 0; i < pc_size; i++) {
		indices.push_back(i);
	}
	GLuint dataEBO;
	glGenBuffers(1, &dataEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, dataEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(),
			&indices[0], GL_STATIC_DRAW);

	// Also set instance data
	// Bind position
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, dataVBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
			(GLvoid*) 0);
	glVertexAttribDivisor(0, 1); // Tell OpenGL this is an instanced vertex attribute.

	// Bind color
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
			(GLvoid*) (3 * sizeof(GLfloat) * (pc_size + normal_pc_size)));
	glVertexAttribDivisor(1, 1); // Tell OpenGL this is an instanced vertex attribute.

	//glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	RenderStruct renderStruct = { };
	renderStruct.VAO = VAO;
	renderStruct.num = pc_size;
	renderStruct.primitive = GLRenderer::points;
	renderStruct.shader = pcShader;
	renderStruct.instanced = true;
	renderStruct.VBO = dataVBO;
	renderStruct.EBO = dataEBO;
	renderStruct.type = GLRenderer::Data;
	this->renderStructs.push_back(renderStruct);

	indices.clear();
	for (int i = 0; i < normal_pc_size; i++) {
		//int originalIndex = normalIndices[i];
		indices.push_back(normalIndices[i]);
		indices.push_back(pc_size + i);
	}

	GLuint normalVAO;
	glGenVertexArrays(1, &normalVAO);
	glBindVertexArray(normalVAO);

	GLuint normalEBO;
	glGenBuffers(1, &normalEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, normalEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(),
			&indices[0], GL_STATIC_DRAW);

	// Also set instance data
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
			(GLvoid*) 0);
	//glVertexAttribDivisor(0, 1); // Tell OpenGL this is an instanced vertex attribute.

	// Set normal color data
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
			(GLvoid*) (3 * sizeof(GLfloat) * (2 * pc_size + normal_pc_size)));
	glVertexAttribDivisor(1, 2 * normal_pc_size); // Tell OpenGL this is an instanced vertex attribute.

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	pcShader.unUse();

	renderStruct = {};
	renderStruct.VAO = normalVAO;
	renderStruct.num = 2 * normal_pc_size;
	renderStruct.primitive = GLRenderer::lines;
	renderStruct.shader = pcShader;
	renderStruct.instanced = true;
	renderStruct.VBO = dataVBO;
	renderStruct.EBO = normalEBO;
	renderStruct.type = GLRenderer::Normal;
	this->renderStructs.push_back(renderStruct);
}

long int GLRenderer::getNanoCount() {
	return (static_cast<long int>(std::chrono::duration_cast<
			std::chrono::nanoseconds>(
			std::chrono::high_resolution_clock::now().time_since_epoch()).count()));
	//return std::chrono::high_resolution_clock::now().time_since_epoch().count();
	/*timeb tb;
	 ftime(&tb);
	 int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
	 return nCount;*/
}

long int GLRenderer::getNanoSpan(long int nTimeStart, long int nTimeEnd) {
	return nTimeEnd - nTimeStart;
	/*int nSpan = nTimeEnd - nTimeStart;
	 if (nSpan < 0)
	 nSpan += 0x100000 * 1000;
	 return nSpan;*/
}

void GLRenderer::doMovement(GLfloat deltaTime) {
	// Camera controls
	if (keys[GLFW_KEY_W]) {
		camera.processKeyboard(FORWARD, deltaTime);
	}
	if (keys[GLFW_KEY_S])
		camera.processKeyboard(BACKWARD, deltaTime);
	if (keys[GLFW_KEY_A])
		camera.processKeyboard(LEFT, deltaTime);
	if (keys[GLFW_KEY_D])
		camera.processKeyboard(RIGHT, deltaTime);
	if (keys[GLFW_KEY_P]) {
		std::cout << "View matrix: " << camera.getViewMatrix() << std::endl;
	}
	if (keys[GLFW_KEY_EQUAL]) {
		this->pointSize += 1;
		if (this->pointSize > 64)
			this->pointSize = 64;
	}
	if (keys[GLFW_KEY_MINUS]) {
		this->pointSize -= 1;
		if (this->pointSize < 1) {
			this->pointSize = 1;
		}
	}
	if (keys[GLFW_KEY_R]) {
		camera.processKeyboard(RESET, deltaTime);
	}
	if (keys[GLFW_KEY_G]) {
		selectPoints = !selectPoints;
	}
	if (keys[GLFW_KEY_N]) {
		showNormals = !showNormals;
	}
}
