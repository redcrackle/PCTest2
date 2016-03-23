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
#include <pcl/common/common.h>
#include <pcl/point_types.h>

//template<typename genType>

namespace glm {
namespace detail {
std::ostream& operator<<(std::ostream& out, const glm::vec2 g);
std::ostream& operator<<(std::ostream& out, const glm::vec3 g);
std::ostream& operator<<(std::ostream& out, const glm::mat4 g);
}
}

extern std::shared_ptr<spdlog::logger> console;

class Utils {
public:
	static std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> applyPassthroughFilter(
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
			Eigen::Vector4f direction, float minThreshold, float maxThreshold,
			std::string filter = "inliers");
	static std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> applyPassthroughFilter(
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, char direction,
			float minThreshold, float maxThreshold, std::string filter =
					"inliers");
};

#endif /* UTILS_HPP_ */
