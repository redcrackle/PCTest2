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
	void record();
	HistPeak getLowestPeak();
};

#endif /* HISTOGRAM_HPP_ */
