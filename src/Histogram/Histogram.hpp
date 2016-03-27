/*
 * Histogram.hpp
 *
 *  Created on: Mar 13, 2016
 *      Author: neeravbm
 */

#ifndef HISTOGRAM_HPP_
#define HISTOGRAM_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

struct HistPeak {
	float low;
	float high;
};

class Histogram {

private:
	float spacing, minValue, maxValue;
	std::vector<float> values, binBoundaries;
	std::vector<int> counts;
	int numberOfBins;
	void calculateMinMaxValues();
	void calculateBinBoundaries();

public:

	Histogram(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
			int numberOfBins = 500, std::string direction = "y");
	Histogram(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
			int numberOfBins = 500,
			Eigen::Vector4f direction = Eigen::Vector4f(0.0f, 1.0f, 0.0f,
					0.0f));
	void record();
	HistPeak getLowestPeak();

	void getMinMaxValues(float* minValue, float* maxValue);
};

#endif /* HISTOGRAM_HPP_ */
