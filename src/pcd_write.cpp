#ifndef GLFW_INCLUDE_GL3
#define GLFW_INCLUDE_GL3 true
#endif

//#define PCL_NO_PRECOMPILE
#include <GL/gl3w.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/octree/octree_pointcloud.h>
#include <Eigen/SVD>
#include <Shader.hpp>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/random.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <FreeImage.h>
#include "camera.cpp"
#include <GLRenderer.hpp>
#include <Utils.hpp>
#include <getopt.h>
#include <RMFE.hpp>
#include <Histogram.hpp>
#include <Histogram2D.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm> //(needed for std::find)#include <math.h>

//int windowWidth = 500, windowHeight = 500; //Width/Height of OpenGL window
Camera camera(glm::vec3(0.0f, 1.0f, 3.0f));
Shader lightShader, lampShader, pcShader;
int N = 10;
int level = spdlog::level::info;

/*struct MyPointType {
 PCL_ADD_POINT4D
 ;                  // preferred way of adding a XYZ+padding
 float test;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
 }EIGEN_ALIGN16;
 // enforce SSE padding for correct memory alignment

 POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType, // here we assume a XYZ + "test" (as fields)
 (float, x, x) (float, y, y) (float, z, z) (float, test, test))*/

FIBITMAP *loadImage(const char *filename) {
	FIBITMAP *dib1 = NULL;
	FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(filename);
	dib1 = FreeImage_Load(fif, filename, JPEG_DEFAULT);
	if (!dib1) {
		std::cerr << "Error opening the image: " << filename << std::endl;
		exit(1);
	}
	return dib1;
}

GLuint loadTexture(FIBITMAP * dib1) {
	GLuint tex_id = 0;
	int x, y, height, width;

	RGBQUAD rgbquad;

	FREE_IMAGE_TYPE type;
	BITMAPINFOHEADER *header;

	type = FreeImage_GetImageType(dib1);
	height = FreeImage_GetHeight(dib1);
	width = FreeImage_GetWidth(dib1);

	header = FreeImage_GetInfoHeader(dib1);
	int scanLineWidth =
			((3 * width) % 4 == 0) ? 3 * width : ((3 * width) % 4) * 4 + 4;
	unsigned char* texels = (GLubyte*) calloc(height * scanLineWidth,
			sizeof(GLubyte));
	for (x = 0; x < width; x++) {
		for (y = 0; y < height; y++) {
			FreeImage_GetPixelColor(dib1, x, y, &rgbquad);
			texels[(y * scanLineWidth + 3 * x)] = ((GLubyte*) &rgbquad)[2];
			texels[(y * scanLineWidth + 3 * x) + 1] = ((GLubyte*) &rgbquad)[1];
			texels[(y * scanLineWidth + 3 * x) + 2] = ((GLubyte*) &rgbquad)[0];
		}
	}

	glGenTextures(1, &tex_id);
	glBindTexture(GL_TEXTURE_2D, tex_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
	GL_UNSIGNED_BYTE, texels);
	glGenerateMipmap(GL_TEXTURE_2D);

	free(texels);
	glBindTexture(GL_TEXTURE_2D, 0);

	return tex_id;
}

void renderPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	GLRenderer glRenderer = GLRenderer(800, 600, glm::vec3(1.0f, 1.0f, 1.0f));
	glRenderer.addAxes();
	glRenderer.addPointCloud(cloud);
	glRenderer.render();
}

void preprocessPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		std::string baseFilename, std::string extension) {

	console->info() << "Removing outliers";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
	console->debug() << "Original points: " << cloud->points.size();
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_filtered);
	console->debug() << "New points: " << cloud_filtered->points.size();

	console->info()
			<< "Scaling the point cloud so that it fits within 1 x 1 x 1 cube";
	pcl::PointXYZRGBNormal min_pt, max_pt;
	pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);
	console->debug() << "Min pt before scaling: " << min_pt;
	console->debug() << "Max pt before scaling: " << max_pt;
	float vol = (max_pt.x - min_pt.x) * (max_pt.y - min_pt.y)
			* (max_pt.z - min_pt.z);
	GLfloat xScale = 1 / (max_pt.x - min_pt.x);
	GLfloat yScale = 1 / (max_pt.y - min_pt.y);
	GLfloat zScale = 1 / (max_pt.z - min_pt.z);
	GLfloat scale = 1; //std::min(xScale, std::min(yScale, zScale));
	GLfloat xOffset = (max_pt.x + min_pt.x) * scale / 2;
	//GLfloat yOffset = 0; //(max_pt.y + min_pt.y) * scale / 2;
	GLfloat yOffset = (max_pt.y + min_pt.y) * scale / 2;
	GLfloat zOffset = (max_pt.z + min_pt.z) * scale / 2;
	for (size_t i = 0; i < cloud_filtered->points.size(); i++) {
		cloud_filtered->points[i].x = scale * cloud_filtered->points[i].x
				- xOffset;
		cloud_filtered->points[i].y = -scale * cloud_filtered->points[i].y
				+ yOffset;
		cloud_filtered->points[i].z = scale * cloud_filtered->points[i].z
				- zOffset;
	}
	vol = vol / std::pow(scale, 3);
	pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);
	console->debug() << "Min pt after scaling: " << min_pt;
	console->debug() << "Max pt after scaling: " << max_pt;
	/*renderPC(cloud_filtered);
	 std::string outputFilename1 = baseFilename + "_processed.pcd";
	 console->info() << "Writing points to " << outputFilename1;
	 pcl::io::savePCDFileASCII(outputFilename1, *cloud_filtered);
	 return;*/

	console->info() << "Rotating the point cloud";
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 0) = -0.882733;
	transform(0, 1) = -0.027756;
	transform(0, 2) = -0.469055;
	transform(1, 0) = -0.469036;
	transform(1, 1) = -0.007611;
	transform(1, 2) = 0.883147;
	transform(2, 0) = -0.028083;
	transform(2, 1) = 0.999586;
	transform(2, 2) = -0.006300;
	transform.transposeInPlace();
	pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform);

	console->info() << "Converting to Voxel grid";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_voxelized(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg;
	vg.setInputCloud(cloud_filtered);
	vg.setLeafSize(0.005f, 0.005f, 0.005f);
	vg.setDownsampleAllData(true);
	vg.filter(*cloud_voxelized);
	console->debug() << "Number of voxels: " << cloud_voxelized->points.size();

	console->info() << "Calculating surface normals";
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
	ne.setInputCloud(cloud_voxelized);
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03f);
	ne.compute(*cloud_normals);
	console->debug() << "Number of normals calculated: "
			<< cloud_normals->points.size();

	console->info() << "Concatenating the clouds";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_processed(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_voxelized, *cloud_normals, *cloud_processed);

	console->info() << "Removing undefined normals";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_defined_normals(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<int> ind;
	pcl::removeNaNNormalsFromPointCloud(*cloud_processed,
			*cloud_defined_normals, ind);
	console->debug() << "Number of defined normals: "
			<< cloud_defined_normals->points.size();

	std::string outputFilename = baseFilename + "_processed.pcd";
	console->info() << "Writing points to " << outputFilename;
	//pcl::io::savePCDFileASCII(outputFilename, *cloud_defined_normals);
	pcl::io::savePCDFileBinary(outputFilename, *cloud_defined_normals);

	console->info() << "Rendering";
	renderPC(cloud_defined_normals);
}

std::string getSceneString() {
	std::cout << "Enter the scene number between 1 and 34" << std::endl;
	std::string scene;
	std::cin >> scene;
	return scene;
}

bool filenameEndsWith(std::string filename, std::string ending) {
	return (filename.compare(filename.length() - ending.length(),
			ending.length(), ending) == 0);
}

int readPCFromFile(std::string filename,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	console->info() << "Trying to read scene from file " << filename;

	std::string error;
	if (filenameEndsWith(filename, ".pcd")) {
		if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
			error = "Could not read the file " + filename + "\n";
			PCL_ERROR(error.c_str());
			return -1;
		}
	} else if (filenameEndsWith(filename, ".ply")) {
		if (pcl::io::loadPLYFile(filename, *cloud) == -1) {
			error = "Could not read the file " + filename + "\n";
			PCL_ERROR(error.c_str());
			return -1;
		}
	} else if (filenameEndsWith(filename, ".obj")) {
		std::ifstream file(filename);
		std::string line;
		while (std::getline(file, line)) {
			if (line.substr(0, 2) == "v ") {
				std::vector<std::string> strs;
				boost::split(strs, line, boost::is_any_of("\t "));
				pcl::PointXYZRGBNormal* pt = new pcl::PointXYZRGBNormal;
				pt->x = std::atof(strs[1].c_str());
				pt->y = std::atof(strs[2].c_str());
				pt->z = std::atof(strs[3].c_str());
				pt->r = (uint8_t) (std::atof(strs[4].c_str()) * 255.0f);
				pt->g = (uint8_t) (std::atof(strs[5].c_str()) * 255.0f);
				pt->b = (uint8_t) (std::atof(strs[6].c_str()) * 255.0f);
				cloud->push_back(*pt);
			}
		}
		return 0;
		if (pcl::io::loadOBJFile(filename, *cloud) == -1) {
			error = "Could not read the file " + filename + "\n";
			PCL_ERROR(error.c_str());
			return -1;
		}
	} else {
		error = "Could not identify the extension of the file " + filename
				+ "\n";
		PCL_ERROR(error.c_str());
		return -1;
	}
	console->info() << "Scene has been read";

	return 0;
}

void extractFloor(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_floor,
		float* corners, unsigned char* &texels, int* floor_size) {
		//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor) {

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Filter all the points whose abs(normal_y) > 0.9.
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i = 0; i < cloud->points.size(); i++) {
		if (std::abs(cloud->points[i].normal_y) > 0.9) {
			inliers->indices.push_back(i);
		}
	}

	// Extract the inliers.
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_filtered);

	Histogram* hist = new Histogram(cloud_filtered, 500, "y");
	hist->record();
	HistPeak peak = hist->getLowestPeak();
	console->debug() << "Floor lies between " << peak.low << " and "
			<< peak.high;

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds_vector =
			Utils::applyPassthroughFilter(cloud, 'y', peak.low, peak.high,
					"both");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor_with_outliers(
			clouds_vector[0]);
	*cloud_without_floor += *clouds_vector[1];

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr floorInliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold((peak.high - peak.low) / 2);
	seg.setInputCloud(cloud_floor_with_outliers);
	seg.segment(*floorInliers, *coefficients);
	console->info() << "Floor plane coefficients: " << coefficients->values[0]
			<< " " << coefficients->values[1] << " " << coefficients->values[2]
			<< " " << coefficients->values[3];

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extractFloorInliers;
	extractFloorInliers.setInputCloud(cloud_floor_with_outliers);
	extractFloorInliers.setIndices(floorInliers);
	extractFloorInliers.setNegative(false);
	extractFloorInliers.filter(*cloud_floor);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	extractFloorInliers.setNegative(true);
	extractFloorInliers.filter(*cloud_floor_outliers);

	*cloud_without_floor += *cloud_floor_outliers;

	Histogram2D* hist2d = new Histogram2D(cloud_floor, 0.01f, 'y', true);
	hist2d->record();
	texels = hist2d->getTexels();
	std::vector<float> xBinBoundaries = hist2d->getXBinBoundaries();
	std::vector<float> zBinBoundaries = hist2d->getZBinBoundaries();
	int numX = xBinBoundaries.size() - 1, numZ = zBinBoundaries.size() - 1;
	float minX = xBinBoundaries[0], maxX = xBinBoundaries[numX - 1], minZ = zBinBoundaries[0], maxZ = zBinBoundaries[numZ - 1];

	float yIntersect = -coefficients->values[3] / coefficients->values[1];
	corners[0] = corners[3] = minX;
	corners[2] = corners[11] = minZ;
	corners[6] = corners[9] = maxX;
	corners[5] = corners[8] = maxZ;
	corners[1] = corners[4] = corners[7] = corners[10] = yIntersect;

	floor_size[0] = numX;
	floor_size[1] = numZ;

	/*console->info() << "Texels filled";

	GLuint tex_id = 0;
	console->info() << "Generating texture";
	glGenTextures(1, &tex_id);
	glBindTexture(GL_TEXTURE_2D, tex_id);
	console->info() << "Tex id: " << tex_id;
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, numX, numZ, 0, GL_RGB, GL_UNSIGNED_BYTE, texels);
	console->info() << "Generating minmap";
	glGenerateMipmap(GL_TEXTURE_2D);

	free(texels);
	glBindTexture(GL_TEXTURE_2D, 0);*/


	/*corners[] = {
			minX, yIntersect, minZ,
			minX, yIntersect, maxZ,
			maxX, yIntersect, maxZ,
			maxX, yIntersect, minZ
	};*/

	//return tex_id;
}

void applyPassthroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		Eigen::Vector4f direction, float minThreshold, float maxThreshold) {

}

void alignPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned) {
	console->info() << "Starting Manhattan Frame Estimation (MFE)";
	// Paper: Robust Manhattan Frame Estimation from a Single RGB-D Image
	if (level <= spdlog::level::debug) {
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal1(
				new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::copyPointCloud(*cloud, *cloud_normal1);

		for (int i = 0; i < cloud_normal1->points.size(); i++) {
			cloud_normal1->points[i].x = cloud_normal1->points[i].normal_x;
			cloud_normal1->points[i].y = cloud_normal1->points[i].normal_y;
			cloud_normal1->points[i].z = cloud_normal1->points[i].normal_z;
		}

		console->info() << "Rendering";
		renderPC(cloud_normal1);

		cloud_normal1->clear();
	}

	RMFE* rmfe = new RMFE(cloud);
	rmfe->alignAxes(*cloud_aligned);

	if (level <= spdlog::level::debug) {
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal2(
				new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::copyPointCloud(*cloud_aligned, *cloud_normal2);

		for (int i = 0; i < cloud_normal2->points.size(); i++) {
			cloud_normal2->points[i].x = cloud_normal2->points[i].normal_x;
			cloud_normal2->points[i].y = cloud_normal2->points[i].normal_y;
			cloud_normal2->points[i].z = cloud_normal2->points[i].normal_z;
		}

		console->info() << "Rendering";
		renderPC(cloud_normal2);

		cloud_normal2->clear();
	}
}

bool extractWall(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered,
		float yThreshold,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_walls,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_wall) {

	// First get all the points that are above the mid-height level.
	pcl::PointXYZRGBNormal min_pt, max_pt;
	pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);

	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds_vector =
			Utils::applyPassthroughFilter(cloud_filtered, 'y', yThreshold,
					max_pt.y, "inliers");
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered_above_yThreshold(
			clouds_vector[0]);

	// cloud_filtered_above_yThreshold should only have high points whose normals are close to XZ plane. Only these points can possibly lie on a wall.
	Histogram2D *hist2d = new Histogram2D(cloud_filtered_above_yThreshold,
			100000);
	std::vector<std::vector<int>> counts = hist2d->record();
	int minCount = hist2d->getMinCount();
	int maxCount = hist2d->getMaxCount();
	std::vector<float> xBinBoundaries = hist2d->getXBinBoundaries();
	std::vector<float> zBinBoundaries = hist2d->getZBinBoundaries();

	int xBins = counts.size(), zBins = counts[0].size();
	cv::Mat img = Utils::getImageFromHistogram2D(counts, minCount, maxCount);

	// Find the edges.
	std::vector<cv::Vec4i> lines = Utils::getEdges(img);

	if (!lines.size()) {
		cloud_filtered->swap(*cloud_without_walls);
		return false;
	}

	// Find the line and point indices for the point which is farthest from the center.
	int farthest_line_index, farthest_point_index;
	Utils::getFarthestPointFromCenter(lines, ((float) xBins) / 2,
			((float) zBins) / 2, &farthest_line_index, &farthest_point_index);

	// Find the point indices for the farthest point and its complement.
	int farthest_point_x_index, farthest_point_z_index,
			complement_point_x_index, complement_point_z_index;
	Utils::getFarthestAndComplementPointsFromIndices(lines, farthest_line_index,
			farthest_point_index, &farthest_point_x_index,
			&farthest_point_z_index, &complement_point_x_index,
			&complement_point_z_index);

	Eigen::RowVector3f line_normal = Utils::getNormalFromLineEndpoints(
			xBinBoundaries, zBinBoundaries, farthest_point_x_index,
			farthest_point_z_index, complement_point_x_index,
			complement_point_z_index);

	pcl::PointXYZRGBNormal seed_point = Utils::getSeedIndex(
			cloud_filtered_above_yThreshold, xBinBoundaries, zBinBoundaries,
			farthest_point_x_index, farthest_point_z_index,
			complement_point_x_index, complement_point_z_index, line_normal);

	pcl::PointIndices::Ptr wall_inliers_indices(new pcl::PointIndices());
	Utils::growRegion(cloud_filtered, seed_point, line_normal,
			wall_inliers_indices);

	// Now that we have wall inlier indices, fit a plane.
	Eigen::Vector4f plane_parameters;
	float curvature;
	pcl::computePointNormal(*cloud_filtered, wall_inliers_indices->indices,
			plane_parameters, curvature);
	// Forcibly make the plane perpendicular to XZ plane. Assuming that all walls are perpendicular to XZ plane.
	plane_parameters(1) = 0;

	double theta = atan2(
			((double) -plane_parameters(3)) / ((double) plane_parameters(2)),
			((double) -plane_parameters(3)) / ((double) plane_parameters(0)));
	console->info() << "Theta of the wall before rotation: " << theta * 180 / M_PI;

	console->info() << "Rotating the point cloud so that the wall is parallel to the X-axis";
	Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
	R(0, 0) = cos(theta);
	R(0, 2) = -sin(theta);
	R(2, 0) = sin(theta);
	R(2, 2) = cos(theta);
	pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, R);

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract_wall_inliers;
	extract_wall_inliers.setInputCloud(cloud_filtered);
	extract_wall_inliers.setIndices(wall_inliers_indices);
	extract_wall_inliers.setNegative(false);
	extract_wall_inliers.filter(*cloud_wall);
	console->info() << "cloud_wall: " << cloud_wall->points.size();

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_wall_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	extract_wall_inliers.setNegative(true);
	extract_wall_inliers.filter(*cloud_wall_outliers);
	//extract_wall_inliers.filter(*cloud_without_walls);
	console->info() << "cloud_wall_outliers: " << cloud_wall_outliers->points.size();

	// Wall is parallel to the X-axis. Find the min and max co-ordinates in X and Z directions.
	pcl::getMinMax3D(*cloud_wall, min_pt, max_pt);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_wall_inliers_among_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// Apply cropbox filter.
	// Initialize filter with extract_removed_indices set to true.
	pcl::CropBox<pcl::PointXYZRGBNormal> crop(true);
	Eigen::Vector4f min_pt_vec, max_pt_vec;
	// Set y to range from -inf to +inf since we want the full wall in vertical direction.
	min_pt_vec << min_pt.x, -1000000, min_pt.z, 1;
	max_pt_vec << max_pt.x, 1000000, max_pt.z, 1;
	crop.setInputCloud(cloud_wall_outliers);
	crop.setMin(min_pt_vec);
	crop.setMax(max_pt_vec);
	crop.filter(*cloud_wall_inliers_among_outliers);

	console->info() << "cloud_wall_inliers_among_outliers: " << cloud_wall_inliers_among_outliers->points.size();
	*cloud_wall += *cloud_wall_inliers_among_outliers;
	console->info() << "cloud_wall: " << cloud_wall->points.size();
	//console->info() << "cloud_without_walls: " << cloud_without_walls->points.size();

	pcl::PointIndices::Ptr cloud_without_wall_indices(new pcl::PointIndices());
	crop.getRemovedIndices(*cloud_without_wall_indices);
	console->info() << "cloud_without_wall_indices: " << cloud_without_wall_indices->indices.size();
	console->info() << "cloud_wall_outliers: " << cloud_wall_outliers->points.size();

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract_wall_inliers_among_outliers;
	extract_wall_inliers.setInputCloud(cloud_wall_outliers);
	extract_wall_inliers.setIndices(cloud_without_wall_indices);
	extract_wall_inliers.setNegative(false);
	extract_wall_inliers.filter(*cloud_without_walls);
	console->info() << "cloud_without_walls: " << cloud_without_walls->points.size();

	R.transposeInPlace();
	pcl::transformPointCloud(*cloud_wall, *cloud_wall, R);
	pcl::transformPointCloud(*cloud_without_walls, *cloud_without_walls, R);

	return true;
}

void extractWalls(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_walls,
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
				Eigen::aligned_allocator<
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> &clouds_walls) {

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Filter out all the points whose abs(normal_y) > 0.10.
	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
	for (int i = 0; i < cloud->points.size(); i++) {
		if (std::abs(cloud->points[i].normal_y) > 0.1) {
			outliers->indices.push_back(i);
		}
	}

	// Filter out the outliers.
	extract.setInputCloud(cloud);
	extract.setIndices(outliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	// First get all the points that are above the mid-height level.
	pcl::PointXYZRGBNormal min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);
	float yThreshold = 0.25 * min_pt.y + 0.75 * max_pt.y;

	// May need to pass cloud instead of cloud_filtered later once we incorporate RANSAC Plane finding for getting the wall.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_wall;

	bool success;
	do {
		cloud_wall.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		cloud_without_walls->clear();
		success = extractWall(cloud_filtered, yThreshold, cloud_without_walls,
				cloud_wall);
		if (success) {
			cloud_filtered->swap(*cloud_without_walls);
			clouds_walls.push_back(cloud_wall);
		}
	} while (success);
}

void processPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {

	console->info() << "Aligning point cloud with axes";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	alignPC(cloud, cloud_aligned);

	//GLRenderer glRenderer = GLRenderer(800, 600, glm::vec3(1.0f, 1.0f, 1.0f));

	console->info() << "Finding the floor";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_floor(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	/*pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);*/
	float floorCorners[12];
	unsigned char* texels;
	int floorSize[2];
	extractFloor(cloud_aligned, cloud_without_floor, floorCorners, texels, floorSize); //cloud_floor);

	console->info() << "Finding walls";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_walls(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
			Eigen::aligned_allocator<
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>> cloud_walls;
	extractWalls(cloud_without_floor, cloud_without_walls, cloud_walls);
	console->info() << "Number of walls found: " << cloud_walls.size();

	console->info() << "Removing outliers from non-floor and walls";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_walls_and_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
	console->debug() << "Original points: "
			<< cloud_without_walls->points.size();
	sor.setInputCloud(cloud_without_walls);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_without_walls_and_outliers);
	console->debug() << "New points: "
			<< cloud_without_walls_and_outliers->points.size();

	console->info() << "Rendering";
	GLRenderer glRenderer = GLRenderer(800, 600, glm::vec3(1.0f, 1.0f, 1.0f));
	glRenderer.addAxes();
	glRenderer.addPointCloud(cloud_without_walls_and_outliers);
	//glRenderer.addPointCloud(cloud_floor, glm::vec3(0.0f, 1.0f, 0.0f));
	glRenderer.addQuad(floorCorners, (GLubyte*) texels, floorSize);
	for (int i = 0; i < cloud_walls.size(); i++) {
		glRenderer.addPointCloud(cloud_walls[i], glm::vecRand3(0.0f, 1.0f));
	}
	glRenderer.render();

	//console->info() << "Writing points to " << file;
	//pcl::io::savePCDFileASCII(outputFilename, *cloud_defined_normals);
	//pcl::io::savePCDFileBinary(file, *cloud_filtered);
}

int main(int argc, char** argv) {
	std::shared_ptr<spdlog::logger> logger = spdlog::get("console");

	const struct option longopts[] = { { "log", 1, 0, 'l' }, { 0, 0, 0, 0 } };
	int c;
	spdlog::set_level(spdlog::level::info);
	bool preprocess = false;
	int index;
	while ((c = getopt_long(argc, argv, "pl:", longopts, &index)) != -1) {
		switch (c) {
		case 'l':
			if (strcmp(optarg, "debug") == 0) {
				level = spdlog::level::debug;
				spdlog::set_level(spdlog::level::debug);
				logger->debug() << "Debug level set";
			} else if (strcmp(optarg, "trace") == 0) {
				level = spdlog::level::trace;
				spdlog::set_level(spdlog::level::trace);
				logger->trace() << "Trace level set";
			}
			break;
		case 'p':
			// Need to run preprocess operation.
			preprocess = true;
			break;
		case '?':
			if (optopt == 'l') {
				std::cerr << "Option --level/-l requires an argument."
						<< std::endl;
				return -1;
			}
			break;
		default:
			abort();
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	std::string scene = getSceneString();

	/*std::string baseFilename = "/Users/neeravbm/Dropbox/ar/pc_scenes/scene"
	 + scene + "_ascii" + (preprocess ? "" : "_processed");*/
	std::string baseFilename =
			"/Users/neeravbm/Dropbox/ar/tango-scenes/dining_room_" + scene
					+ (preprocess ? "" : "_processed");
	std::string extension = "pcd";
	//std::string extension = (preprocess ? "obj" : "pcd");
	std::string file = baseFilename + "." + extension;
	//string file = "/Users/neeravbm/Dropbox/ar/data/segmentation/mOSD/test/test" + scene_str + ".pcd";
	//std::string file = "/Users/neeravbm/Dropbox/ar/rgbd-scenes-v2/pc/"
	//		+ scene_str + ".ply";
	char const* filename = file.c_str();

	bool error = readPCFromFile(filename, cloud);
	if (error != 0) {
		return error;
	}

	if (preprocess) {
		preprocessPC(cloud, baseFilename, extension);
	} else {
		processPC(cloud);
	}

	return 0;
}

