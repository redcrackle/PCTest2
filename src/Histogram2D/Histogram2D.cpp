/*
 * Histogram2D.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: neeravbm
 */

#include <Histogram2D.hpp>

Histogram2D::Histogram2D(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		int numberOfBins) {
	this->numberOfBins = numberOfBins;

	std::vector<float> vals(2);
	for (int i = 0; i < cloud->points.size(); i++) {
		vals.clear();
		vals.push_back(cloud->points[i].x);
		vals.push_back(cloud->points[i].z);
		this->values.push_back(vals);
	}

	this->calculateMinMaxValues();
	this->calculateBinBoundaries();

	std::vector<int> row;
	for (int x = 0; x < this->xBinBoundaries.size() - 1; x++) {
		row.clear();
		for (int z = 0; z < this->zBinBoundaries.size() - 1; z++) {
			row.push_back(0);
		}
		this->counts.push_back(row);
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
	this->spacing = std::sqrt((this->xMaxValue - this->xMinValue) * (this->zMaxValue - this->zMinValue)  / this->numberOfBins);
	float numberOfXBins = (int) ((this->xMaxValue - this->xMinValue) / this->spacing) + 1;
	float numberOfZBins = (int) ((this->zMaxValue - this->zMinValue) / this->spacing) + 1;

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
	for (int i = 0; i < this->values.size(); i++) {
		int xBin = (int) ((this->values[i][0] - this->xMinValue) / this->spacing);
		int zBin = (int) ((this->values[i][1] - this->zMinValue) / this->spacing);
		this->counts[xBin][zBin] += 1;
	}

	// Calculate min and max counts.
	std::vector<int> row;
	int currentCount;
	this->minCount = this->maxCount = this->counts[0][0];
	for (int x = 1; x < this->xBinBoundaries.size() - 1; x++) {
		for (int z = 1; z < this->zBinBoundaries.size() - 1; z++) {
			currentCount = this->counts[x][z];
			if (currentCount < this->minCount) {
				this->minCount = currentCount;
			}
			else if (currentCount > this->maxCount) {
				this->maxCount = currentCount;
			}
		}
	}

	return this->counts;
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
