/*
 * Histogram2D.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: neeravbm
 */

#include <Histogram2D.hpp>

Histogram2D::Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		int numberOfBins, char normalDirection, bool texturize) {
	this->numberOfBins = numberOfBins;
	this->texturize = texturize;

	std::vector<float> vals(2);
	switch (normalDirection) {
	case 'y':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].x);
			vals.push_back(cloud->points[i].z);
			this->values.push_back(vals);
		}
		break;
	case 'x':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].y);
			vals.push_back(cloud->points[i].z);
			this->values.push_back(vals);
		}
		break;
	case 'z':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].x);
			vals.push_back(cloud->points[i].y);
			this->values.push_back(vals);
		}
		break;
	}

	this->calculateMinMaxValues();
	this->spacing = std::sqrt(
			(this->xMaxValue - this->xMinValue)
					* (this->zMaxValue - this->zMinValue) / this->numberOfBins);
	this->calculateBinBoundaries();

	std::vector<int> row;
	int numX = this->xBinBoundaries.size() - 1, numZ = this->zBinBoundaries.size() - 1;
	for (int x = 0; x < numX; x++) {
		row.clear();
		for (int z = 0; z < numZ; z++) {
			row.push_back(0);
		}
		this->counts.push_back(row);
	}

	if (this->texturize) {
		std::vector<float> colorVals(3);
		for (int i = 0; i < cloud->points.size(); i++) {
			colorVals.clear();
			colorVals.push_back(cloud->points[i].r);
			colorVals.push_back(cloud->points[i].g);
			colorVals.push_back(cloud->points[i].b);
			this->color.push_back(colorVals);
		}

		this->colorSum = (unsigned int*) calloc(numX * numZ * 3, sizeof(unsigned int));
	}
}

Histogram2D::Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		float spacing, char normalDirection, bool texturize) {
	this->spacing = spacing;
	this->texturize = texturize;

	std::vector<float> vals(2);
	switch (normalDirection) {
	case 'y':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].x);
			vals.push_back(cloud->points[i].z);
			this->values.push_back(vals);
		}
		break;
	case 'x':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].y);
			vals.push_back(cloud->points[i].z);
			this->values.push_back(vals);
		}
		break;
	case 'z':
		for (int i = 0; i < cloud->points.size(); i++) {
			vals.clear();
			vals.push_back(cloud->points[i].x);
			vals.push_back(cloud->points[i].y);
			this->values.push_back(vals);
		}
		break;
	}

	this->calculateMinMaxValues();
	this->calculateBinBoundaries();

	std::vector<int> row;
	int numX = this->xBinBoundaries.size() - 1, numZ = this->zBinBoundaries.size() - 1;
	for (int x = 0; x < numX; x++) {
		row.clear();
		for (int z = 0; z < numZ; z++) {
			row.push_back(0);
		}
		this->counts.push_back(row);
	}

	if (this->texturize) {
		std::vector<float> colorVals(3);
		for (int i = 0; i < cloud->points.size(); i++) {
			colorVals.clear();
			colorVals.push_back(cloud->points[i].r);
			colorVals.push_back(cloud->points[i].g);
			colorVals.push_back(cloud->points[i].b);
			this->color.push_back(colorVals);
		}

		this->colorSum = (unsigned int*) calloc(numX * numZ * 3, sizeof(unsigned int));
	}
}

void Histogram2D::calculateMinMaxValues() {
	// Find min and max values.
	this->xMinValue = this->xMaxValue = this->values[0][0];
	this->zMinValue = this->zMaxValue = this->values[0][1];
	float xValue, zValue;
	for (int i = 1; i < this->values.size(); i++) {
		xValue = this->values[i][0];
		if (xValue > this->xMaxValue) {
			this->xMaxValue = xValue;
		} else if (xValue < this->xMinValue) {
			this->xMinValue = xValue;
		}

		zValue = this->values[i][1];
		if (zValue > this->zMaxValue) {
			this->zMaxValue = zValue;
		} else if (zValue < this->zMinValue) {
			this->zMinValue = zValue;
		}
	}
}

void Histogram2D::calculateBinBoundaries() {
	float numberOfXBins = (int) ((this->xMaxValue - this->xMinValue)
			/ this->spacing) + 1;
	float numberOfZBins = (int) ((this->zMaxValue - this->zMinValue)
			/ this->spacing) + 1;

	this->xBinBoundaries.push_back(this->xMinValue);
	float currentXVal = this->xMinValue;
	while (currentXVal < this->xMaxValue) {
		currentXVal += this->spacing;
		this->xBinBoundaries.push_back(currentXVal);
	}

	this->zBinBoundaries.push_back(this->zMinValue);
	float currentZVal = this->zMinValue;
	while (currentZVal < this->zMaxValue) {
		currentZVal += this->spacing;
		this->zBinBoundaries.push_back(currentZVal);
	}
}

std::vector<std::vector<int>> Histogram2D::record() {
	int numX = this->xBinBoundaries.size() - 1, numZ =
			this->zBinBoundaries.size() - 1, base_index;

	for (int i = 0; i < this->values.size(); i++) {
		int xBin =
				(int) ((this->values[i][0] - this->xMinValue) / this->spacing);
		int zBin =
				(int) ((this->values[i][1] - this->zMinValue) / this->spacing);
		this->counts[xBin][zBin] += 1;

		if (texturize) {
			base_index = (xBin * numZ + zBin) * 3;
			colorSum[base_index] += this->color[i][0];
			colorSum[base_index + 1] += this->color[i][1];
			colorSum[base_index + 2] += this->color[i][2];
		}
	}

	// Calculate min and max counts.
	std::vector<int> row;
	int currentCount;
	this->minCount = this->maxCount = this->counts[0][0];
	for (int x = 1; x < numX; x++) {
		for (int z = 1; z < numZ; z++) {
			currentCount = this->counts[x][z];
			if (currentCount < this->minCount) {
				this->minCount = currentCount;
			} else if (currentCount > this->maxCount) {
				this->maxCount = currentCount;
			}
		}
	}

	if (texturize) {
		int count = 0;
		texels = (unsigned char*) calloc(numX * numZ * 3, sizeof(unsigned char));
		for (int x = 0; x < numX; x++) {
			for (int z = 0; z < numZ; z++) {
				count = this->counts[x][z];
				if (count > 0) {
					base_index = (x * numZ + z) * 3;
					texels[base_index] = (unsigned char) (colorSum[base_index] / count);
					texels[base_index + 1] = (unsigned char) (colorSum[base_index + 1] / count);
					texels[base_index + 2] = (unsigned char) (colorSum[base_index + 2] / count);
				}
			}
		}
	}

	return this->counts;
}

unsigned char* Histogram2D::getTexels() {
	return this->texels;
}

int Histogram2D::getMinCount() {
	return this->minCount;
}

int Histogram2D::getMaxCount() {
	return this->maxCount;
}

std::vector<float> Histogram2D::getXBinBoundaries() {
	return this->xBinBoundaries;
}

std::vector<float> Histogram2D::getZBinBoundaries() {
	return this->zBinBoundaries;
}
