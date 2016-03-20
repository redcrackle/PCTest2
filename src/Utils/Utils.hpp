/*
 * Utils.hpp
 *
 *  Created on: Jan 15, 2016
 *      Author: neeravbm
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <functional>
#include <spdlog/spdlog.h>

//template<typename genType>

namespace glm {
namespace detail {
std::ostream& operator<<(std::ostream& out, const glm::vec2 g);
std::ostream& operator<<(std::ostream& out, const glm::vec3 g);
std::ostream& operator<<(std::ostream& out, const glm::mat4 g);
}
}

extern std::shared_ptr<spdlog::logger> console;

#endif /* UTILS_HPP_ */
