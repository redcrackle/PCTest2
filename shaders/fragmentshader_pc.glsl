#version 410 core

in vec3 fColor;

out vec4 color;

void main() {
	//color = vec4(0.5f, 0.5f, 0.5f, 0.5f);
	color = vec4(fColor, 1.0f);
}
