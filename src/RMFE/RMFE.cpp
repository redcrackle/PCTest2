/*
 * RMFE.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: neeravbm
 */

#include <RMFE.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

RMFE::RMFE() {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	this->cloud_filtered_normals = cloud_filtered;
}

RMFE::RMFE(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	this->cloud = cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	this->cloud_filtered_normals = cloud_filtered;
}

void RMFE::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	this->cloud = cloud;
}

void RMFE::alignAxes(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out) {
	this->filterNormals();
	Eigen::Matrix3d R = this->getRotationMatrix();
	this->rotatePointCloud(R);
	cloud_out = *this->cloud;
}

void RMFE::rotatePointCloud(Eigen::Matrix3d R) {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 0) = R(0, 0);
	transform(0, 1) = R(0, 1);
	transform(0, 2) = R(0, 2);
	transform(1, 0) = R(1, 0);
	transform(1, 1) = R(1, 1);
	transform(1, 2) = R(1, 2);
	transform(2, 0) = R(2, 0);
	transform(2, 1) = R(2, 1);
	transform(2, 2) = R(2, 2);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_rotated(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloudWithNormals(*this->cloud, *cloud_rotated,
			transform);
	this->cloud = cloud_rotated;
}

void RMFE::filterNormals() {
	pcl::IndicesPtr indices = this->getRelevantNormalIndices();
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> eifilter;
	eifilter.setInputCloud(this->cloud);
	eifilter.setIndices(indices);
	eifilter.filter(*this->cloud_filtered_normals);
}

pcl::IndicesPtr RMFE::getRelevantNormalIndices() {
	std::vector<int> indices;
	for (size_t i = 0; i < this->cloud->points.size(); i++) {
		if (std::abs(this->cloud->points[i].normal_y) >= 0.9
				|| std::abs(this->cloud->points[i].normal_y) <= 0.1) {
			indices.push_back(i);
		}
	}
	boost::shared_ptr<std::vector<int> > indicesptr(
			new std::vector<int>(indices));
	return indicesptr;
}

Eigen::MatrixXd RMFE::getNormalMatrix() {
	Eigen::MatrixXd normalMatrix(3, this->cloud_filtered_normals->points.size());
	for (int i = 0; i < this->cloud_filtered_normals->points.size(); i++) {
		normalMatrix.col(i) << this->cloud_filtered_normals->points[i].normal_x, this->cloud_filtered_normals->points[i].normal_y, this->cloud_filtered_normals->points[i].normal_z;
	}

	return normalMatrix;
}

void RMFE::applySoftThresholding(Eigen::MatrixXd &input, double lambda) {
	for (int i = 0; i < input.rows(); i++) {
		for (int j = 0; j < input.cols(); j++) {
			double v = input(i, j);
			int sign = (input(i, j) > 0) - (input(i, j) < 0);
			input(i, j) = sign * std::max(0.0, std::abs(v) - lambda);
		}
	}
}

void RMFE::applyQFunction(Eigen::MatrixXd &input, double lambda) {
	for (int i = 0; i < input.rows(); i++) {
		long double rowL2Sum = 0;
		for (int j = 0; j < input.cols(); j++) {
			rowL2Sum += std::pow((double) input(i, j), 2.0);
		}
		rowL2Sum = std::sqrt(rowL2Sum);
		double factor = std::max(0.0, 1 - lambda / (double) rowL2Sum);
		for (int j = 0; j < input.cols(); j++) {
			input(i, j) *= factor;
		}
	}
}

Eigen::Matrix3d RMFE::getRotationMatrix() {
	// Paper: Robust Manhattan Frame Estimation from a Single RGB-D Image
	double lambda = 0.3;
	Eigen::MatrixXd N = this->getNormalMatrix();
	Eigen::Matrix3d R = Eigen::Matrix<double, 3, 3>::Identity();

	float last_error = 10000.0f, error_diff = 10000.0f;
	while (error_diff > 0.0001 || error_diff < 0) {
		Eigen::MatrixXd X = R * N;
		RMFE::applySoftThresholding(X, lambda);
		Eigen::MatrixXd XNTranspose = X * N.transpose();
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(XNTranspose,
				Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd U = svd.matrixU();
		Eigen::MatrixXd V = svd.matrixV();
		Eigen::VectorXd D = svd.singularValues();

		/*std::cout << "U: " << U << std::endl;
		 std::cout << "V: " << V << std::endl;
		 std::cout << "D: " << D << std::endl;*/

		//std::cout << "Remaining: " << (U * D.asDiagonal() * V.transpose() - XNTranspose) << std::endl;
		Eigen::Matrix3d S = Eigen::Matrix<double, 3, 3>::Identity();
		if (XNTranspose.determinant() < 0) {
			S(2, 2) = -1;
		}

		R = U * S * V.transpose();

		float L1NormSum = 0;
		for (int j = 0; j < X.rows(); j++) {
			L1NormSum += X.row(j).lpNorm<1>();
		}
		float error = 0.5 * std::pow((R * N - X).lpNorm<2>(), 2)
				+ lambda * L1NormSum;

		error_diff = last_error - error;
		last_error = error;

		/*std::cout << "Iteration: " << iter << std::endl;
		 std::cout << "R: " << R << std::endl;
		 //std::cout << "X: " << X << std::endl;
		 std::cout << "Error: " << error << std::endl;*/
	}

	return R;
}
