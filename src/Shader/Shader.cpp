#ifndef GLFW_INCLUDE_GL3
#define GLFW_INCLUDE_GL3 true
#endif

#include <fstream>
#include <iostream>
#include <string>
#include <GL/gl3w.h>
#include <Shader.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace std;

Shader::Shader() {}

Shader::Shader(const char *vertexShaderFilename, const char *FragmentShaderFilename) {
	GLuint vertexShader = this->initshaders(GL_VERTEX_SHADER, vertexShaderFilename);
	GLuint fragmentShader = this->initshaders(GL_FRAGMENT_SHADER, FragmentShaderFilename);
	GLuint program = glCreateProgram();
	shaderProgram = this->initprogram(vertexShader, fragmentShader);
	GLint linked;
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linked);
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

void Shader::use() {
	glUseProgram(this->shaderProgram);
}

void Shader::unUse() {
	glUseProgram(0);
}

void Shader::setParam3f(const char *param, GLfloat param1, GLfloat param2, GLfloat param3) {
	glUniform3f(glGetUniformLocation(this->shaderProgram, param), param1, param2, param3);
}

void Shader::setParamMatrix4fv(const char *param, glm::mat4 mat) {
	glUniformMatrix4fv(glGetUniformLocation(this->shaderProgram, param), 1, GL_FALSE, glm::value_ptr(mat));
}

void Shader::setParamli(const char *param, GLint li) {
	glUniform1i(glGetUniformLocation(this->shaderProgram, param), li);
}

void Shader::setParam1f(const char *param, GLfloat val) {
	glUniform1i(glGetUniformLocation(this->shaderProgram, param), val);
}

GLuint Shader::initshaders (GLenum type, const char *filename) {
	// Using GLSL shaders, OpenGL book, page 679
	GLuint shader = glCreateShader(type) ;
	GLint compiled ;
	string str = textFileRead (filename) ;
	GLchar * cstr = new GLchar[str.size()+1] ;
	const GLchar * cstr2 = cstr ; // Weirdness to get a const char
	std::strcpy(cstr,str.c_str()) ;
	glShaderSource (shader, 1, &cstr2, NULL) ;
	glCompileShader (shader) ;
	glGetShaderiv (shader, GL_COMPILE_STATUS, &compiled) ;
	if (!compiled) {
		shadererrors (shader) ;
		throw 3 ;
	}
	return shader ;
}

GLuint Shader::initprogram (GLuint vertexshader, GLuint fragmentshader) {
	GLuint program = glCreateProgram() ;
	GLint linked ;
	glAttachShader(program, vertexshader) ;
	glAttachShader(program, fragmentshader) ;
	glLinkProgram(program) ;
	glGetProgramiv(program, GL_LINK_STATUS, &linked) ;
	if (linked) glUseProgram(program) ;
	else {
		programerrors(program) ;
		throw 4 ;
	}
	return program;
}

string Shader::textFileRead (const char * filename) {
	string str, ret = "" ;
	ifstream in ;
	in.open(filename) ;
	if (in.is_open()) {
		getline (in, str) ;
		while (in) {
			ret += str + "\n" ;
			getline (in, str) ;
		}
		//    cout << "Shader below\n" << ret << "\n" ;
		return ret ;
	}
	else {
		cerr << "Unable to Open File " << filename << "\n" ;
		throw 2 ;
	}
}

void Shader::programerrors (const GLint program) {
	GLint length ;
	GLchar * log ;
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length) ;
	log = new GLchar[length+1] ;
	glGetProgramInfoLog(program, length, &length, log) ;
	cout << "Compile Error, Log Below\n" << log << "\n" ;
	delete [] log ;
}

void Shader::shadererrors (const GLint shader) {
	GLint length ;
	GLchar * log ;
	glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length) ;
	log = new GLchar[length+1] ;
	glGetShaderInfoLog(shader, length, &length, log) ;
	cout << "Compile Error, Log Below\n" << log << "\n" ;
	delete [] log ;
}

/*class Shader {

public:
	GLuint shaderProgram;

	Shader() {}

	Shader(const char *vertexShaderFilename, const char *FragmentShaderFilename) {
		GLuint vertexShader = this->initshaders(GL_VERTEX_SHADER, vertexShaderFilename);
		GLuint fragmentShader = this->initshaders(GL_FRAGMENT_SHADER, FragmentShaderFilename);
		GLuint program = glCreateProgram();
		shaderProgram = this->initprogram(vertexShader, fragmentShader);
		GLint linked;
		glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linked);
		glDeleteShader(vertexShader);
		glDeleteShader(fragmentShader);
	}

	void use() {
		glUseProgram(this->shaderProgram);
	}

	void unUse() {
		glUseProgram(0);
	}

	void setParam3f(const char *param, GLfloat param1, GLfloat param2, GLfloat param3) {
		glUniform3f(glGetUniformLocation(this->shaderProgram, param), param1, param2, param3);
	}

	void setParamMatrix4fv(const char *param, glm::mat4 mat) {
		glUniformMatrix4fv(glGetUniformLocation(this->shaderProgram, param), 1, GL_FALSE, glm::value_ptr(mat));
	}

	void setParamli(const char *param, GLint li) {
		glUniform1i(glGetUniformLocation(this->shaderProgram, param), li);
	}

	void setParam1f(const char *param, GLfloat val) {
		glUniform1i(glGetUniformLocation(this->shaderProgram, param), val);
	}


private:
	GLuint initshaders (GLenum type, const char *filename) {
		// Using GLSL shaders, OpenGL book, page 679 
		GLuint shader = glCreateShader(type) ; 
		GLint compiled ; 
		string str = textFileRead (filename) ; 
		GLchar * cstr = new GLchar[str.size()+1] ; 
		const GLchar * cstr2 = cstr ; // Weirdness to get a const char
		strcpy(cstr,str.c_str()) ; 
		glShaderSource (shader, 1, &cstr2, NULL) ; 
		glCompileShader (shader) ; 
		glGetShaderiv (shader, GL_COMPILE_STATUS, &compiled) ; 
		if (!compiled) { 
			shadererrors (shader) ; 
			throw 3 ; 
		}
		return shader ; 
	}

	GLuint initprogram (GLuint vertexshader, GLuint fragmentshader) {
		GLuint program = glCreateProgram() ; 
		GLint linked ; 
		glAttachShader(program, vertexshader) ; 
		glAttachShader(program, fragmentshader) ; 
		glLinkProgram(program) ; 
		glGetProgramiv(program, GL_LINK_STATUS, &linked) ; 
		if (linked) glUseProgram(program) ; 
		else { 
			programerrors(program) ; 
			throw 4 ; 
		}
		return program; 
	}

	string textFileRead (const char * filename) {
		string str, ret = "" ; 
		ifstream in ; 
		in.open(filename) ; 
		if (in.is_open()) {
			getline (in, str) ; 
			while (in) {
				ret += str + "\n" ; 
				getline (in, str) ; 
			}
			//    cout << "Shader below\n" << ret << "\n" ; 
			return ret ; 
		}
		else {
			cerr << "Unable to Open File " << filename << "\n" ; 
			throw 2 ; 
		}
	}

	void programerrors (const GLint program) {
		GLint length ; 
		GLchar * log ; 
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length) ; 
		log = new GLchar[length+1] ;
		glGetProgramInfoLog(program, length, &length, log) ; 
		cout << "Compile Error, Log Below\n" << log << "\n" ; 
		delete [] log ; 
	}

	void shadererrors (const GLint shader) {
		GLint length ; 
		GLchar * log ; 
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length) ; 
		log = new GLchar[length+1] ;
		glGetShaderInfoLog(shader, length, &length, log) ; 
		cout << "Compile Error, Log Below\n" << log << "\n" ; 
		delete [] log ; 
	}
};*/
