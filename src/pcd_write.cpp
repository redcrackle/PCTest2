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

int windowWidth = 500, windowHeight = 500; //Width/Height of OpenGL window
GLuint VAO, lampVAO, VBO, VAO2, VBO2, EBO, EBO2, texture1, texture2, VAO_pc,
		VBO_pc;
glm::mat4 trans, model, view, projection;
glm::vec3 cubePositions[10], rotationAxes[10], cameraPos = glm::vec3(0.0f, 0.0f,
		3.0f), cameraFront = glm::vec3(0.0f, 0.0f, -1.0f), cameraUp = glm::vec3(
		0.0f, 1.0f, 0.0f);
GLfloat mix = 0.5, deltaTime = 0.0f, lastX = 400.0f, lastY = 300.0f, yaw = 0.0f,
		pitch = 0.0f, fov = 45.0f;
bool keys[1024];
int lastFrame;
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
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor) {

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

	Histogram* hist = new Histogram(cloud_filtered);
	hist->record();
	HistPeak peak = hist->getLowestPeak();
	console->debug() << "Floor lies between " << peak.low << " and "
			<< peak.high;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor_with_outliers(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(peak.low, peak.high);
	pass.filter(*cloud_floor_with_outliers);

	// Get the point cloud without the floor.
	pass.setFilterLimitsNegative(true);
	pass.filter(*cloud_without_floor);

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
	console->debug() << "Floor plane coefficients: " << coefficients->values[0]
			<< " " << coefficients->values[1] << " " << coefficients->values[2]
			<< " " << coefficients->values[3];

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

void extractWalls(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_walls,
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds_walls) {

	// First get all the points that are above the mid-height level.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_above_yThreshold(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointXYZRGBNormal min_pt, max_pt;
	pcl::getMinMax3D(*cloud, min_pt, max_pt);
	float yThreshold = (min_pt.y + max_pt.y) / 2;
	pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(yThreshold, max_pt.y);
	pass.filter(*cloud_above_yThreshold);

	// Fiter out all the points whose absolute normals in y direction are more than 0.1.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	// Filter all the points whose abs(normal_y) > 0.9.
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

	// cloud_filtered should only have high points who normals are close to XZ plane. Only these points can possibly lie on a wall.
	Histogram2D *hist2d = new Histogram2D(cloud_filtered, 1000);
	std::vector<std::vector<int>> counts = hist2d->record();
	int minCount = hist2d->getMinCount();
	int maxCount = hist2d->getMaxCount();

}

void processPC(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {

	console->info() << "Aligning point cloud with axes";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	alignPC(cloud, cloud_aligned);

	console->info() << "Finding the floor";
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_without_floor(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_floor(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	extractFloor(cloud_aligned, cloud_without_floor, cloud_floor);

	console->info() << "Rendering";
	GLRenderer glRenderer = GLRenderer(800, 600, glm::vec3(1.0f, 1.0f, 1.0f));
	glRenderer.addAxes();
	glRenderer.addPointCloud(cloud_without_floor);
	glRenderer.addPointCloud(cloud_floor, glm::vec3(0.0f, 1.0f, 0.0f));
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

