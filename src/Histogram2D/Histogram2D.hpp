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
	std::vector<std::vector<float>> color;
	std::vector<float> xBinBoundaries, zBinBoundaries;
	std::vector<std::vector<int>> counts;
	unsigned int* colorSum;
	int numberOfBins, minCount, maxCount;
	bool texturize;
	unsigned char* texels;
	void calculateMinMaxValues();
	void calculateBinBoundaries();

public:

	Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, int numberOfBins = 500, char normalDirection = 'y', bool texturize = false);
	Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, float spacing = 0.01f, char normalDirection = 'y', bool texturize = false);
	std::vector<std::vector<int> > record();
	int getMinCount();
	int getMaxCount();
	std::vector<float> getXBinBoundaries();
	std::vector<float> getZBinBoundaries();
	unsigned char* getTexels();
};

#endif /* HISTOGRAM2D_HPP_ */
