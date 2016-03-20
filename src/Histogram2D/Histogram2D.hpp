/*
 * Histogram2D.hpp
 *
 *  Created on: Mar 19, 2016
 *      Author: neeravbm
 */

#ifndef HISTOGRAM2D_HPP_
#define HISTOGRAM2D_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

class Histogram2D {

private:
	float spacing, xMinValue, zMinValue, xMaxValue, zMaxValue;
	std::vector<std::vector<float>> values;
	std::vector<float> xBinBoundaries, zBinBoundaries;
	std::vector<std::vector<int>> counts;
	int numberOfBins, minCount, maxCount;
	void calculateMinMaxValues();
	void calculateBinBoundaries();

public:

	Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
			int numberOfBins = 500);
	std::vector<std::vector<int> > record();
	int getMinCount();
	int getMaxCount();
	std::vector<float> getXBinBoundaries();
	std::vector<float> getZBinBoundaries();
};

#endif /* HISTOGRAM2D_HPP_ */
