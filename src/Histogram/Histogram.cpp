/*
 * Histogram.cpp
 *
 *  Created on: Mar 13, 2016
 *      Author: neeravbm
 */

#include <Histogram.hpp>

Histogram::Histogram(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		int numberOfBins, std::string direction) {
	this->numberOfBins = numberOfBins;

	for (int i = 0; i < cloud->points.size(); i++) {
		this->values.push_back(cloud->points[i].y);
	}

	for (int i = 0; i < numberOfBins; i++) {
		this->counts.push_back(0);
	}

	this->calculateMinMaxValues();
	this->calculateBinBoundaries();

}

void Histogram::calculateMinMaxValues() {
	// Find min and max values.
	this->minValue = this->maxValue = this->values[0];
	float value;
	for (int i = 1; i < this->values.size(); i++) {
		value = this->values[i];
		if (value > this->maxValue) {
			this->maxValue = value;
		} else if (value < this->minValue) {
			this->minValue = value;
		}
	}
}

void Histogram::calculateBinBoundaries() {
	this->spacing = (this->maxValue - this->minValue) / this->numberOfBins;
	/*this->binBoundaries.push_back(this->minValue);
	for (int i = 1; i < this->numberOfBins; i++) {
		this->binBoundaries.push_back(this->binBoundaries[i] + this->spacing);
	}
	this->binBoundaries.push_back(this->maxValue);*/
}

void Histogram::record() {
	for (int i = 0; i < this->values.size(); i++) {
		int num = (int) ((this->values[i] - this->minValue) / this->spacing);
		if (num < 0) {
			num = 0;
		}
		if (num > this->counts.size() - 1) {
			num = this->counts.size() - 1;
		}
		this->counts[num] = this->counts[num] + 1;
	}
}

HistPeak Histogram::getLowestPeak() {
	// Increase count from 0 and count width as from half the peak to half the peak.
	float currentLowIndex = 0;
	float currentHighIndex = 0;
	float currentMax = this->counts[0];
	for (int i = 1; i < this->counts.size(); i++) {
		if (this->counts[i] > currentMax) {
			currentMax = this->counts[i];
			currentLowIndex = i;
			for (int j = i; j >= 0; j--) {
				if (this->counts[j] < 0.1 * currentMax) {
					currentLowIndex = j;
					break;
				}
			}
		}
		else if (this->counts[i] < 0.1 * currentMax) {
			currentHighIndex = i;
			break;
		}
	}

	HistPeak s;
	s.low = this->minValue + this->spacing * currentLowIndex;
	s.high = this->minValue + this->spacing * currentHighIndex;

	return s;
}
