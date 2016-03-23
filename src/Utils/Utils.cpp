#include <iostream>
#include <Utils.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

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

std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Utils::applyPassthroughFilter(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		Eigen::Vector4f direction, float minThreshold, float maxThreshold,
		std::string filter) {

	// http://docs.pointclouds.org/1.7.1/classpcl_1_1_point_cloud.html#a6ff67b42a2596e5edd53713c51cd8ce4
	Eigen::MatrixXf pointsTranspose = cloud->getMatrixXfMap(3, 4, 0);
	Eigen::MatrixXf points = pointsTranspose.transpose();
	Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(points.rows(), 1);
	Eigen::MatrixXf homogeneousPoints(points.rows(), 4);
	homogeneousPoints << points, ones;

	Eigen::VectorXf vectorSum = homogeneousPoints * direction;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_inliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (int i = 0; i < vectorSum.rows(); i++) {
		if (vectorSum(i) >= minThreshold && vectorSum(i) <= maxThreshold) {
			inliers->indices.push_back(i);
		}
	}

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds_filtered;

	// Filter out the outliers.
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);

	if (filter == "both" || filter == "inliers") {
		extract.setNegative(false);
		extract.filter(*cloud_inliers);
		clouds_filtered.push_back(cloud_inliers);
	}

	if (filter == "both" || filter == "outliers") {
		extract.setNegative(false);
		extract.filter(*cloud_outliers);
		clouds_filtered.push_back(cloud_outliers);
	}

	return clouds_filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Utils::applyPassthroughFilter(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, char direction, float minThreshold,
		float maxThreshold, std::string filter) {

	pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
	pass.setInputCloud(cloud);
	std::string str(1, direction);
	pass.setFilterFieldName(str);
	pass.setFilterLimits(minThreshold, maxThreshold);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_inliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds_filtered;

	if (filter == "both" || filter == "inliers") {
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_inliers);
		clouds_filtered.push_back(cloud_inliers);
	}

	if (filter == "both" || filter == "outliers") {
		pass.setFilterLimitsNegative(true);
		pass.filter(*cloud_outliers);
		clouds_filtered.push_back(cloud_outliers);
	}

	return clouds_filtered;
}
