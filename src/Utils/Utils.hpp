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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

	static std::vector<cv::Vec4i> getEdges(cv::Mat img);

	static cv::Mat getImageFromHistogram2D(std::vector<std::vector<int>> counts,
			int minCount, int maxCount);

	static void getFarthestPointFromCenter(std::vector<cv::Vec4i> lines,
			float centerX, float centerZ, int* farthest_line_index,
			int* farthest_point_index);

	static void getFarthestAndComplementPointsFromIndices(
			std::vector<cv::Vec4i> lines, int farthest_line_index,
			int farthest_point_index, int* farthest_point_x_index,
			int* farthest_point_z_index, int* complement_point_x_index,
			int* complement_point_z_index);

	static Eigen::RowVector3f getNormalFromLineEndpoints(
			std::vector<float> xBinBoundaries,
			std::vector<float> zBinBoundaries, int farthest_point_x_index,
			int farthest_point_z_index, int complement_point_x_index,
			int complement_point_z_index);

	static pcl::PointXYZRGBNormal getSeedIndex(
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered_above_yThreshold,
			std::vector<float> xBinBoundaries,
			std::vector<float> zBinBoundaries, int farthest_point_x_index,
			int farthest_point_z_index, int complement_point_x_index,
			int complement_point_z_index, Eigen::RowVector3f line_normal);

	static void growRegion(
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered,
			pcl::PointXYZRGBNormal seed_point, Eigen::RowVector3f line_normal,
			pcl::PointIndices::Ptr wall_inliers_indices);

};

#endif /* UTILS_HPP_ */
