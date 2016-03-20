/*
 * RMFE.hpp
 *
 *  Created on: Jan 31, 2016
 *      Author: neeravbm
 */
#pragma once

#ifndef RMFE_HPP_
#define RMFE_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

class RMFE {
public:
	RMFE();
	RMFE(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
	void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
	void alignAxes(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);

private:
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered_normals;
	Eigen::Matrix3d getRotationMatrix();
	void filterNormals();
	Eigen::MatrixXd getNormalMatrix();
	pcl::IndicesPtr getRelevantNormalIndices();
	static void applySoftThresholding(Eigen::MatrixXd &input, double lambda);
	static void applyQFunction(Eigen::MatrixXd &input, double lambda);
	void rotatePointCloud(Eigen::Matrix3d R);
};

#endif /* RMFE_HPP_ */
