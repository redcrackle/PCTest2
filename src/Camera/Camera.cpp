// GL Includes
#include <GL/gl3w.h>
#include <Camera.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
//#include <glm/gtx/rotate_vector.hpp>
#include <iostream>

// Constructor with vectors
Camera::Camera(glm::vec3 position, glm::vec3 up, GLfloat yaw, GLfloat pitch) :
		Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(
				SENSITIVTY), Zoom(ZOOM) {
	this->logger = spdlog::get("console");
	this->Position = position;
	this->WorldUp = up;
	this->Yaw = yaw;
	this->Pitch = pitch;
	this->updateCameraVectors();
}
// Constructor with scalar values
Camera::Camera(GLfloat posX, GLfloat posY, GLfloat posZ, GLfloat upX,
		GLfloat upY, GLfloat upZ, GLfloat yaw, GLfloat pitch) :
		Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(
				SENSITIVTY), Zoom(ZOOM) {
	this->logger = spdlog::get("console");
	this->Position = glm::vec3(posX, posY, posZ);
	this->WorldUp = glm::vec3(upX, upY, upZ);
	this->Yaw = yaw;
	this->Pitch = pitch;
	this->updateCameraVectors();
}

// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
glm::mat4 Camera::getViewMatrix() {
	return glm::lookAt(this->Position, this->Position + this->Front, this->Up);
}

void Camera::reset() {
	this->Position = glm::vec3(0.0f, 0.0f, 3.0f);
	this->Up = glm::vec3(0.0f, 1.0f, 0.0f);
	this->Front = glm::vec3(0.0f, 0.0f, -1.0f);
	this->Right = glm::vec3(1.0f, 0.0f, 0.0f);
}

// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
void Camera::processKeyboard(Camera_Movement direction, GLfloat deltaTime) {
	GLfloat velocity = this->MovementSpeed * deltaTime;
	logger->debug() << "Movement velocity: " << velocity;
	if (direction == FORWARD) {
		logger->debug() << "Moving forward by " << this->Front * velocity;
		this->Position += this->Front * velocity;
	}
	if (direction == BACKWARD) {
		logger->debug() << "Moving backward by " << this->Front * velocity;
		this->Position -= this->Front * velocity;
	}
	if (direction == LEFT) {
		logger->debug() << "Moving to the left by " << this->Right * velocity;
		this->Position -= this->Right * velocity;
	}
	if (direction == RIGHT) {
		logger->debug() << "Moving to the right by " << this->Right * velocity;
		this->Position += this->Right * velocity;
	}
	if (direction == RESET) {
		logger->debug() << "Resetting the view to the starting position";
		this->reset();
	}
}

// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
void Camera::processMouseMovement(GLfloat xoffset, GLfloat yoffset,
		GLboolean constrainPitch) {
	xoffset *= this->MouseSensitivity;
	yoffset *= this->MouseSensitivity;

	logger->debug() << "Offset: " << glm::vec2(xoffset, yoffset);
	logger->debug() << "Old position: " << this->Position;

	glm::vec3 mouseMovementVector = yoffset * this->Up + xoffset * this->Right;
	logger->debug() << "Mouse movement vector: " << mouseMovementVector;

	glm::vec3 rotationAxis = glm::normalize(glm::cross(this->Front, mouseMovementVector));
	logger->debug() << "Rotation axis: " << rotationAxis;

	GLfloat angle = std::sqrt(std::pow(xoffset, 2) + std::pow(yoffset, 2));
	logger->debug() << "Angle: " << angle;

	glm::mat4 rotationMatrix = glm::mat4();
	rotationMatrix = glm::rotate(rotationMatrix, angle, rotationAxis);
	logger->debug() << "Rotation matrix: " << rotationMatrix;

	glm::vec4 rotatedPos = rotationMatrix * glm::vec4(this->Position, 1);
	this->Position = glm::vec3(rotatedPos.x / rotatedPos.w,
			rotatedPos.y / rotatedPos.w, rotatedPos.z / rotatedPos.w);
	logger->debug() << "New position: " << this->Position;

	logger->debug() << "Old up vector" << this->Up;
	glm::vec3 newUp = 0.99f * this->Up + 0.01f * glm::normalize(glm::cross(this->Position, rotationAxis));
        logger->debug() << "New unprocessed up vector" << newUp;
	// We need to find the component of the above vector that is perpendicular to the new position vector.
	this->Up = glm::normalize(newUp - glm::dot(newUp, this->Position) / glm::length2(this->Position) * this->Position);
	//this->Up = glm::normalize(0.99f * this->Up + 0.01f * glm::normalize(glm::cross(this->Position, rotationAxis)));
	logger->debug() << "New up vector" << this->Up;

	logger->debug() << "Old right vector" << this->Right;
	this->Right = glm::normalize(glm::cross(this->Up, this->Position));
	logger->debug() << "New right vector" << this->Right;

	logger->debug() << "Old front vector" << this->Front;
	this->Front = -glm::normalize(this->Position);
	logger->debug() << "New front vector" << this->Front;

	/*std::cout << "Offset: " << xoffset << " " << yoffset << std::endl;
	 GLfloat norm = glm::gtx::norm::l2Norm(this->Position);
	 //GLfloat norm = glm::normalize(this->Position);
	 //glm::vec3 newPosition = this->Position + yoffset * this->Up - xoffset * this->Right;
	 GLfloat newX = this->Position.x + yoffset * this->Up.x
	 - xoffset * this->Right.x;
	 GLfloat newY = this->Position.y + yoffset * this->Up.y
	 - xoffset * this->Right.y;
	 //GLfloat newY = this->Position.y + yoffset;
	 GLfloat newZ = std::sqrt(
	 std::pow(norm, 2) - std::pow(newX, 2) - std::pow(newY, 2));
	 glm::vec3 newPosition = glm::vec3(newX, newY, newZ);
	 //this->Up = - newPosition + this->Position;
	 std::cout << "Old position: " << this->Position.x << " " << this->Position.y
	 << " " << this->Position.z << std::endl;
	 std::cout << "New position: " << newPosition.x << " " << newPosition.y
	 << " " << newPosition.z << std::endl;
	 std::cout << "Old up vector: " << this->Up.x << " " << this->Up.y << " "
	 << this->Up.z << std::endl;
	 this->Up = glm::normalize(newPosition - this->Position);
	 this->Position = newPosition;
	 this->Front = -glm::normalize(this->Position);
	 this->Right = glm::normalize(glm::cross(this->Up, this->Position));
	 this->Up = glm::normalize(glm::cross(this->Right, this->Front));
	 std::cout << "New up vector: " << this->Up.x << " " << this->Up.y << " "
	 << this->Up.z << std::endl;*/

	/*this->Up = this->Yaw += xoffset;
	 this->Pitch += yoffset;

	 // Make sure that when pitch is out of bounds, screen doesn't get flipped
	 if (constrainPitch) {
	 if (this->Pitch > 89.0f)
	 this->Pitch = 89.0f;
	 if (this->Pitch < -89.0f)
	 this->Pitch = -89.0f;
	 }

	 // Update Front, Right and Up Vectors using the updated Eular angles
	 this->updateCameraVectors();*/
}

// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
void Camera::processMouseScroll(GLfloat yoffset) {
	if (this->Zoom >= 1.0f && this->Zoom <= 45.0f)
		this->Zoom -= yoffset;
	if (this->Zoom <= 1.0f)
		this->Zoom = 1.0f;
	if (this->Zoom >= 45.0f)
		this->Zoom = 45.0f;
}

// Calculates the front vector from the Camera's (updated) Eular Angles
void Camera::updateCameraVectors() {
	// Calculate the new Front vector
	glm::vec3 front;
	front.x = cos(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
	front.y = sin(glm::radians(this->Pitch));
	front.z = sin(glm::radians(this->Yaw)) * cos(glm::radians(this->Pitch));
	this->Front = glm::normalize(front);
	// Also re-calculate the Right and Up vector
	this->Right = glm::normalize(glm::cross(this->Front, this->WorldUp)); // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	this->Up = glm::normalize(glm::cross(this->Right, this->Front));
}
