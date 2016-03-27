#include <iostream>
#include <Utils.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <Histogram.hpp>

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
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, char direction,
		float minThreshold, float maxThreshold, std::string filter) {

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

std::vector<cv::Vec4i> Utils::getEdges(cv::Mat img) {
	cv::Mat detected_edges;
	std::vector<cv::Vec4i> lines;
	cv::blur(img, detected_edges, cv::Size(5, 5));
	cv::Canny(detected_edges, detected_edges, 50, 200, 5);
	cv::HoughLinesP(detected_edges, lines, 1, CV_PI / 180, 50, 20, 10);

	cv::Mat detected_lines = cv::Mat::zeros(detected_edges.size().height,
			detected_edges.size().width, CV_8UC3);
	cv::Vec4i l;
	for (int i = 0; i < lines.size(); i++) {
		l = lines[i];
		cv::line(detected_lines, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				CV_RGB(0, 0, 255), 1, CV_AA);
	}

	/*cv::imshow("Detected Lines", detected_lines);
	 cv::waitKey(0);*/

	return lines;
}

cv::Mat Utils::getImageFromHistogram2D(std::vector<std::vector<int>> counts,
		int minCount, int maxCount) {
	cv::Mat img;
	uint8_t color;

	int rows = counts.size();
	int cols = counts[0].size();

	// Vertical image axis corresponds to X and Horizontal image axis corresponds to Z.
	img.create(rows, cols, CV_8UC1);
	for (int x = 0; x < rows; x++) {
		for (int z = 0; z < cols; z++) {
			color = (uint8_t) (((float) (counts[x][z] - minCount))
					/ ((float) (maxCount - minCount)) * 255.0f);
			img.at<uchar>(x, z) = color;
		}
	}

	return img;
}

void Utils::getFarthestPointFromCenter(std::vector<cv::Vec4i> lines,
		float centerX, float centerZ, int* farthest_line_index,
		int* farthest_point_index) {
	*farthest_line_index = 0;
	*farthest_point_index = 0;
	float max_squared_distance = 0.0f;
	float squared_distance = 0.0f;
	float distance_x, distance_z;
	for (int line_index = 0; line_index < lines.size(); line_index++) {
		for (int point_index = 0; point_index <= 2; point_index += 2) {
			distance_z = lines[line_index][point_index] - centerZ;
			distance_x = lines[line_index][point_index + 1] - centerX;
			squared_distance = std::pow(distance_z, 2)
					+ std::pow(distance_x, 2);
			if (squared_distance > max_squared_distance) {
				*farthest_line_index = line_index;
				*farthest_point_index = point_index;
				max_squared_distance = squared_distance;
			}
		}
	}
}

void Utils::getFarthestAndComplementPointsFromIndices(
		std::vector<cv::Vec4i> lines, int farthest_line_index,
		int farthest_point_index, int* farthest_point_x_index,
		int* farthest_point_z_index, int* complement_point_x_index,
		int* complement_point_z_index) {

	// OpenCV Point notation has column first and row second.

	*farthest_point_z_index = lines[farthest_line_index][farthest_point_index];
	*farthest_point_x_index = lines[farthest_line_index][farthest_point_index
			+ 1];

	int complement_point_index = (farthest_point_index == 0) ? 2 : 0;
	*complement_point_z_index =
			lines[farthest_line_index][complement_point_index];
	*complement_point_x_index =
			lines[farthest_line_index][complement_point_index + 1];
}

Eigen::RowVector3f Utils::getNormalFromLineEndpoints(
		std::vector<float> xBinBoundaries, std::vector<float> zBinBoundaries,
		int point1_x_index, int point1_z_index, int point2_x_index,
		int point2_z_index) {

	// Following equation is valid as long as the grid spacing across X and Z axis is the same.
	Eigen::RowVector3f line_normal;
	line_normal
			<< -(zBinBoundaries[point1_z_index] - zBinBoundaries[point2_z_index]), 0, (xBinBoundaries[point1_x_index]
			- xBinBoundaries[point2_x_index]);
	line_normal.normalize();
	console->info() << "Line normal is: " << line_normal(0) << " "
			<< line_normal(1) << " " << line_normal(2);

	return line_normal;
}

pcl::PointXYZRGBNormal Utils::getSeedIndex(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered_above_yThreshold,
		std::vector<float> xBinBoundaries, std::vector<float> zBinBoundaries,
		int farthest_point_x_index, int farthest_point_z_index,
		int complement_point_x_index, int complement_point_z_index,
		Eigen::RowVector3f line_normal) {

	int number_of_seed_points = 0, width = 1, index_x_min, index_x_max,
			index_z_min, index_z_max;
	float farthest_point_x_min, farthest_point_x_max, farthest_point_x_mid,
			farthest_point_z_min, farthest_point_z_max, farthest_point_z_mid,
			normal_product, max_normal_product = 0;
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
			clouds_xThresholded_vector, clouds_xzThresholded_vector;
	Eigen::RowVector3f point_normal;
	int seed_index;
	while (number_of_seed_points <= 0) {
		index_x_min =
				(farthest_point_x_index - width + 1 > 0) ?
						farthest_point_x_index - width + 1 : 0;
		index_x_max =
				(farthest_point_x_index + width < xBinBoundaries.size()) ?
						farthest_point_x_index + width :
						xBinBoundaries.size() - 1;
		farthest_point_x_min = xBinBoundaries[index_x_min];
		farthest_point_x_max = xBinBoundaries[index_x_max];
		farthest_point_x_mid = (farthest_point_x_min + farthest_point_x_max)
				/ 2;

		index_z_min =
				(farthest_point_z_index - width + 1 > 0) ?
						farthest_point_z_index - width + 1 : 0;
		index_z_max =
				(farthest_point_z_index + width < zBinBoundaries.size()) ?
						farthest_point_z_index + width :
						zBinBoundaries.size() - 1;
		farthest_point_z_min = zBinBoundaries[index_z_min];
		farthest_point_z_max = zBinBoundaries[index_z_max];
		farthest_point_z_mid = (farthest_point_z_min + farthest_point_z_max)
				/ 2;

		// Apply passthrough filter to get points in cuboid.
		clouds_xThresholded_vector = Utils::applyPassthroughFilter(
				cloud_filtered_above_yThreshold, 'x', farthest_point_x_min,
				farthest_point_x_max, "inliers");
		number_of_seed_points = clouds_xThresholded_vector[0]->points.size();
		if (number_of_seed_points == 0) {
			width += 1;
			continue;
		}

		clouds_xzThresholded_vector = Utils::applyPassthroughFilter(
				clouds_xThresholded_vector[0], 'z', farthest_point_z_min,
				farthest_point_z_max, "inliers");
		number_of_seed_points = clouds_xzThresholded_vector[0]->points.size();
		if (number_of_seed_points == 0) {
			width += 1;
			continue;
		}

		// Find the point whose normal is the closest to line_normal vector.
		for (int i = 0; i < clouds_xzThresholded_vector[0]->points.size();
				i++) {
			point_normal << clouds_xzThresholded_vector[0]->points[i].normal_x, clouds_xzThresholded_vector[0]->points[i].normal_y, clouds_xzThresholded_vector[0]->points[i].normal_z;
			// It should be possible to improve this code by using cloud->getMatrixXfMap() to get all the normals in matrix form and then multiplying with line normal.
			normal_product = std::abs(
					(float) ((Eigen::VectorXf) (point_normal
							* line_normal.transpose()))(0));
			if (normal_product > max_normal_product
					&& std::abs(normal_product) > 0.95) {
				seed_index = i;
				max_normal_product = normal_product;
			} else {
				number_of_seed_points--;
			}
		}
		if (number_of_seed_points == 0) {
			width += 1;
			continue;
		}
	}

	// We have the seed point now. Start region growing.
	console->info() << "Seed point for region growth: ("
			<< clouds_xzThresholded_vector[0]->points[seed_index].x << ", "
			<< clouds_xzThresholded_vector[0]->points[seed_index].y << ", "
			<< clouds_xzThresholded_vector[0]->points[seed_index].z
			<< ") with normal ("
			<< clouds_xzThresholded_vector[0]->points[seed_index].normal_x
			<< ", "
			<< clouds_xzThresholded_vector[0]->points[seed_index].normal_y
			<< ", "
			<< clouds_xzThresholded_vector[0]->points[seed_index].normal_z
			<< ")";

	return clouds_xzThresholded_vector[0]->points[seed_index];
}

void Utils::growRegion(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered,
		pcl::PointXYZRGBNormal seed_point, Eigen::RowVector3f line_normal,
		pcl::PointIndices::Ptr wall_inliers_indices) {

	pcl::PointIndices::Ptr wall_neighbors_indices(new pcl::PointIndices());

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
	kdtree.setInputCloud(cloud_filtered);
	std::vector<int> pointsIdxRadius;
	std::vector<float> pointsSquaredDistRadius;
	float radius = 0.05f;

	int count = kdtree.radiusSearch(seed_point, radius, pointsIdxRadius,
			pointsSquaredDistRadius);
	// Just add the first point, which will be the same point as the seed.
	wall_neighbors_indices->indices.push_back(pointsIdxRadius[0]);
	wall_inliers_indices->indices.push_back(pointsIdxRadius[0]);

	int current_index = 0;
	float normal_product;

	do {
		//console->info() << "Entering the do loop";
		pointsIdxRadius.clear();
		pointsSquaredDistRadius.clear();
		count =
				kdtree.radiusSearch(
						cloud_filtered->points[wall_inliers_indices->indices[current_index]],
						radius, pointsIdxRadius, pointsSquaredDistRadius);
		//kdtree.nearestKSearch(clouds_xzThresholded_vector[0]->points[seed_index], K, pointsIdxRadius, pointsSquaredDistRadius);
		//console->info() << "Number of neighbors found: " << count;

		for (int i = 0; i < pointsIdxRadius.size(); i++) {
			/*console->info() << "Index: " << i << "; Point Index: " << pointsIdxRadius[i];
			 console->info() << "Existing point indices:";
			 for (int j = 0; j < wall_neighbors_indices->indices.size(); j++) {
			 console->info() << wall_neighbors_indices->indices[j];
			 }*/
			if (pointsSquaredDistRadius[i] > 0
					&& !std::binary_search(
							wall_neighbors_indices->indices.begin(),
							wall_neighbors_indices->indices.end(),
							pointsIdxRadius[i])) {
				/*&& std::find(wall_neighbors_indices->indices.begin(),
				 wall_neighbors_indices->indices.end(),
				 pointsIdxRadius[i])
				 == wall_neighbors_indices->indices.end()) {*/

				/*console->info() << "Point " << i
				 << " has not been encountered before";*/
				// This is a new point we haven't encountered before.
				wall_neighbors_indices->indices.push_back(pointsIdxRadius[i]);
				std::sort(wall_neighbors_indices->indices.begin(),
						wall_neighbors_indices->indices.end());

				normal_product =
						line_normal(0)
								* cloud_filtered->points[pointsIdxRadius[i]].normal_x
								+ line_normal(1)
										* cloud_filtered->points[pointsIdxRadius[i]].normal_y
								+ line_normal(2)
										* cloud_filtered->points[pointsIdxRadius[i]].normal_z;
				/*console->info() << "Normal product: "
				 << std::abs(normal_product);*/
				if (std::abs(normal_product) > 0.90) {
					// Add to the region.
					//console->info() << "Adding to the inliers";
					wall_inliers_indices->indices.push_back(pointsIdxRadius[i]);
				}
			} else {
				/*console->info() << "Point " << i
				 << " has been encountered before or is the same as seed point.";*/
			}
		}

		current_index++;
		/*console->info() << "Current index: " << current_index;
		 console->info() << "Number of inliers: "
		 << wall_inliers_indices->indices.size();*/
	} while (current_index < wall_inliers_indices->indices.size());
}
