#include <iostream>
#include <Utils.hpp>

namespace glm {
namespace detail {
std::ostream& operator<<(std::ostream& out, const glm::vec2 g) {
	return out << glm::to_string(g);
}

std::ostream& operator<<(std::ostream& out, const glm::vec3 g) {
	return out << glm::to_string(g);
}

std::ostream& operator<<(std::ostream& out, const glm::mat4 g) {
	return out << glm::to_string(g);
}
}
}

std::shared_ptr<spdlog::logger> console = spdlog::stdout_logger_mt("console");
