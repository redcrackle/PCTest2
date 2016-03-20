#pragma once

#ifndef GLFW_INCLUDE_GL3
#define GLFW_INCLUDE_GL3 true
#endif

#include <glm/glm.hpp>

#ifndef __INCLUDESHADERS
#define __INCLUDESHADERS

class Shader {

public:
	GLuint shaderProgram;

	Shader();
	Shader(const char *vertexShaderFilename,
			const char *FragmentShaderFilename);
	void use();
	void unUse();
	void setParam3f(const char *param, GLfloat param1, GLfloat param2,
			GLfloat param3);
	void setParamMatrix4fv(const char *param, glm::mat4 mat);
	void setParamli(const char *param, GLint li);
	void setParam1f(const char *param, GLfloat val);

private:
	GLuint initshaders(GLenum type, const char *filename);
	GLuint initprogram(GLuint vertexshader, GLuint fragmentshader);
	std::string textFileRead(const char * filename);
	void programerrors(const GLint program);
	void shadererrors(const GLint shader);
};

#endif 
