#pragma comment(lib,"user32.lib") 
#pragma comment(lib,"gdi32.lib") 

#include <iostream>

#include <time.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING
#include <experimental/filesystem>

#define PCL_NO_PRECOMPILE 1

#define VTK_LEGACY_SILENT

#include <string>
#include <filesystem>
#include <stdio.h>
#include <pcl/filters/uniform_sampling.h>

#include <chrono>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/boundary.h>


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/moment_of_inertia_estimation.h>


#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/features/don.h>

#include <boost/algorithm/string.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

#include <CGAL/IO/File_writer_wavefront.h>
#include <CGAL/IO/generic_copy_OFF.h>
#include <CGAL/Surface_mesh/IO.h>

#include <CGAL/Heat_method_3/Surface_mesh_geodesic_distances_3.h>
#include <pcl/visualization/point_picking_event.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <pcl/features/principal_curvatures.h>

#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <utility> // defines std::pair

#include <pcl/PCLPointCloud2.h>
#include <pcl/console/print.h>

#include <cmath>

#define EPS 2.2204e-16

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <pcl/tracking/normal_coherence.h>
//#include <Windows.h>


using namespace pcl;
using namespace pcl::io;
using namespace std;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#include <pcl/point_cloud.h>

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/Complex_2_in_triangulation_3_file_writer.h>
#include <pcl/filters/random_sample.h>


#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>

#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>

#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <boost/function_output_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel CGAL_Kernal;
//typedef CGAL::Simple_cartesian<float> CGAL_Kernal;
typedef CGAL_Kernal::Point_3 PointCGAL;
typedef CGAL_Kernal::Vector_3 VectorCGAL;
typedef CGAL_Kernal::Compare_dihedral_angle_3 Compare_dihedral_angle_3;

typedef CGAL::Polyhedron_3<CGAL_Kernal> Polyhedron;
typedef boost::graph_traits<Polyhedron>::face_descriptor face_descriptor_poly;

typedef CGAL::Surface_mesh<PointCGAL> Surface_mesh;
typedef Surface_mesh::Vertex_index SM_vertex_descriptor;
typedef boost::graph_traits<Surface_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Surface_mesh>::face_descriptor  face_descriptor;
typedef boost::graph_traits<Surface_mesh>::edge_descriptor  edge_descriptor;
typedef boost::graph_traits<Surface_mesh>::faces_size_type  faces_size_type;
typedef CGAL_Kernal::Point_2                                                 Point_2;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor          halfedge_descriptor;


typedef Surface_mesh::Property_map<vertex_descriptor, double> Vertex_distance_map;

typedef pcl::PointXYZ Point;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;
typedef search::KdTree<PointXYZ>::Ptr KdTreePtr;

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

// Point with normal vector stored in a std::pair.
typedef std::pair<PointCGAL, VectorCGAL> PointVectorPair;
// Concurrency
#ifdef CGAL_LINKED_WITH_TBBf
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>


#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

namespace SMS = CGAL::Surface_mesh_simplification;

struct myclass3 {
	bool operator() (pcl::PointIndices i, pcl::PointIndices j) { return (i.indices.size() > j.indices.size()); }
} myobject3;


struct myclass {
	bool operator() (PointCGAL i, PointCGAL j) { return (i.y() < j.y()); }
} myobject1;


struct myclass2 {
	bool operator() (pcl::PointNormal i, pcl::PointNormal j) { return (i.x < j.x); }
} myobject2;


class Timer
{
private:
	// Type aliases to make accessing nested type easier
	using clock_t = std::chrono::high_resolution_clock;
	using second_t = std::chrono::duration<double, std::ratio<1> >;

	std::chrono::time_point<clock_t> m_beg;

public:
	Timer() : m_beg(clock_t::now())
	{
	}

	void reset()
	{
		m_beg = clock_t::now();
	}

	double elapsed() const
	{
		return std::chrono::duration_cast<second_t>(clock_t::now() - m_beg).count();
	}
};

string meshFile = "";
string pointFile = "";

string outPath = "";
string fileNameOnly = "";

#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <pcl/surface/mls.h>

void uniformSampling(string cloudPath, string cloudOutPath, double cloud_res = 0.1F)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

	pcl::io::loadPLYFile<pcl::PointNormal>(cloudPath, *cloud);

	std::cout << "Loaded :" << cloud->width * cloud->height << std::endl;

	float radius = 0.8F;

	pcl::UniformSampling<pcl::PointNormal> filter;
	filter.setInputCloud(cloud);
	filter.setRadiusSearch(radius);
	filter.filter(*cloud_filtered);

	std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height << std::endl;

	pcl::PLYWriter writer;
	writer.write<pcl::PointNormal>(cloudOutPath, *cloud_filtered, false);
}

void writeMatrix_to_XYZ(Eigen::MatrixXd& src, string fileName, int cols = 3)
{
	ofstream writeStream(fileName, ios::out | ios::trunc);
	if (writeStream)
	{
		for (size_t i = 0; i < src.rows(); i++)
		{
			for (size_t j = 0; j < cols; j++)
			{
				writeStream << src(i, j) << " ";
			}
			writeStream << endl;
		}

		writeStream.close();
	}
	else
	{
		cout << "Error opening file" << endl;
	}
}


// void setInputFile(std::string inputFile)
// {
// 	cout << "\nSelected file: " + inputFile + "\n";
// 	meshFile = inputFile;
// 	std::experimental::filesystem::path filePath = { inputFile };
// 	fileNameOnly = filePath.stem().string().substr(0, 14);
// 	outPath = filePath.parent_path().string() + "/";
// }

void setInputFile(std::string inputFile, std::string tempPath = "")
{
    cout << "\nSelected file: " + inputFile + "\n";
    meshFile = inputFile;
    std::experimental::filesystem::path filePath = { inputFile };
    fileNameOnly = filePath.stem().string().substr(0, 14);
    
    if (!tempPath.empty()) {
        outPath = tempPath;
    } else {
        outPath = filePath.parent_path().string() + "/";
    }
}


bool getNormalsOnSurface(pcl::PointCloud<Point>::Ptr sampledPointCloud, pcl::PointCloud<pcl::PointNormal>::Ptr surfaceWithNormals)
{
	string fragmentMeshFile = "fileForNormalEst";
	Eigen::MatrixXd pointCloudMatrix(sampledPointCloud->points.size(), 3);
	for (size_t i = 0; i < sampledPointCloud->points.size(); i++)
	{
		pointCloudMatrix.row(i) << sampledPointCloud->points[i].x, sampledPointCloud->points[i].y, sampledPointCloud->points[i].z;
	}
	writeMatrix_to_XYZ(pointCloudMatrix, fragmentMeshFile + "_Sampled.xyz", 3);

	string fName = fragmentMeshFile + "_Sampled.xyz";
	const char* fname = fName.c_str();
	// Reads a .xyz point set file in points[].
	std::list<PointVectorPair> points;
	std::ifstream stream(fname);
	if (!stream ||
		!CGAL::read_xyz_points(stream,
			std::back_inserter(points),
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())))
	{
		std::cerr << "Error: cannot read file " << fname << std::endl;
		return false;
	}
	// Estimates normals direction.
	const int nb_neighbors = 5;
	CGAL::pca_estimate_normals<Concurrency_tag>
		(points, nb_neighbors,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
	// Orients normals.
	std::list<PointVectorPair>::iterator unoriented_points_begin =
		CGAL::mst_orient_normals(points, nb_neighbors,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));
	points.erase(unoriented_points_begin, points.end());


	/// Saves point set.
	std::ofstream out(fragmentMeshFile + "_SampledWithNormals.xyz");
	out.precision(17);
	if (!out ||
		!CGAL::write_xyz_points(
			out, points,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
	{
		return false;
	}
	out.close();

	string line;

	stringstream ss;
	ifstream myfile(fragmentMeshFile + "_SampledWithNormals.xyz");
	std::vector<std::string> fileContents;
	std::string str;
	std::string file_contents;
	int numberOfPoints;
	//Reading file contents
	while (std::getline(myfile, str))
	{
		fileContents.push_back(str);
	}

	//Constructing the matrix
	numberOfPoints = fileContents.size();
	surfaceWithNormals->width = numberOfPoints;
	surfaceWithNormals->height = 1;
	surfaceWithNormals->is_dense = false;
	surfaceWithNormals->points.resize(surfaceWithNormals->width * surfaceWithNormals->height);

	std::vector<std::string> result;
	for (int i = 0; i < numberOfPoints; i++) //points.size(); i++)
	{
		std::istringstream iss(fileContents[i]);
		for (std::string s; iss >> s; )
		{
			result.push_back(s);
		}

		surfaceWithNormals->points[i].x = ::atof(result[0].c_str());
		surfaceWithNormals->points[i].y = ::atof(result[1].c_str());
		surfaceWithNormals->points[i].z = ::atof(result[2].c_str());
		surfaceWithNormals->points[i].normal_x = ::atof(result[3].c_str());
		surfaceWithNormals->points[i].normal_y = ::atof(result[4].c_str());
		surfaceWithNormals->points[i].normal_z = ::atof(result[5].c_str());
		result.clear();

	}
	surfaceWithNormals->width = surfaceWithNormals->points.size();
	surfaceWithNormals->height = 1;


	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	for (size_t i = 0; i < numberOfPoints; i++)
	{
		pcl::Normal nrm;
		nrm.normal_x = surfaceWithNormals->points[i].normal_x;
		nrm.normal_y = surfaceWithNormals->points[i].normal_y;
		nrm.normal_z = surfaceWithNormals->points[i].normal_z;

		normals->points.push_back(nrm);
	}
	normals->width = numberOfPoints;
	normals->height = 1;

	// pcl::io::savePLYFile(fragmentMeshFile + "_SampledWithNormals.ply", *surfaceWithNormals);
	return true;
}

#include <pcl/surface/mls.h>

pcl::PointCloud<pcl::PointNormal>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointNormal>);
pcl::UniformSampling<pcl::PointNormal> uniformSampler;
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr downsampledCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
std::string outputFile = "";

std::string downsamplePointCloud(std::string inputPcdFile, float samplingRadius = 0.6F) {
	pcl::PointCloud<pcl::PointNormal>::Ptr originalPointCloud(new pcl::PointCloud<pcl::PointNormal>);
	if (pcl::io::loadPCDFile<pcl::PointNormal>(inputPcdFile, *originalPointCloud) == -1) {
		PCL_ERROR("Couldn't read file\n");
		return ""; 
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr originalCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*originalPointCloud, *originalCloudWithNormals);

	pcl::KdTreeFLANN<pcl::PointNormal> kdTree;
	kdTree.setInputCloud(originalCloudWithNormals);

	uniformSampler.setInputCloud(originalCloudWithNormals);
	uniformSampler.setRadiusSearch(samplingRadius); //downsample_10
	uniformSampler.filter(*downsampledCloud);

	pcl::copyPointCloud(*downsampledCloud, *downsampledCloudPoints);
	getNormalsOnSurface(downsampledCloudPoints, downsampledCloudWithNormals);

	pcl::PLYWriter writer;
	outputFile = outPath + fileNameOnly + "_SampledWithNormals.ply";
	writer.write<pcl::PointNormal>(outputFile, *downsampledCloudWithNormals, false);
	return outputFile;
}

int getClusters_EuclideanDistBased(string fileName)
{
	pcl::PointCloud<PointTypeIO>::Ptr cloud_in(new pcl::PointCloud<PointTypeIO>), cloud_out(new pcl::PointCloud<PointTypeIO>);
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
	pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
	pcl::console::TicToc tt;
	pcl::PointCloud<pcl::PointNormal>::Ptr missedPoints(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PCDWriter writer;
	pcl::io::loadPLYFile(fileName, *missedPoints);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(missedPoints);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
	ec.setClusterTolerance(2); // 2cm
	ec.setMinClusterSize(50); // 100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(missedPoints);
	ec.extract(cluster_indices);

	int noOfClusters = 0;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointNormal>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->push_back((*missedPoints)[*pit]); //*
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
		std::stringstream ss;
		ss << fileName + "Cluster_" << j << ".pcd";
		writer.write<pcl::PointNormal>(ss.str(), *cloud_cluster, false); //*
		j++;
	}
	return j;
}

bool pointExistsInCLoud(pcl::PointXYZ pt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	bool ptExists = false;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].x == pt.x && cloud->points[i].y == pt.y && cloud->points[i].z == pt.z)
		{
			ptExists = true;
		}
	}
	return ptExists;
}


void getPointsInSequence(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sequenced)
{

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(cloud);

	pcl::PointXYZ searchPoint;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<int> indices;

	pcl::PointXYZ currentPoint, nextPoint, previousPoint;


	currentPoint = cloud->points[0];
	cloud_sequenced->points.push_back(currentPoint);
	for (size_t t = 1; t < cloud->points.size(); t++)
	{
		searchPoint = currentPoint;
		int K = 50;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{

			if (t == 1)
			{
				PointXYZ p;
				p.x = (*cloud)[pointIdxNKNSearch[1]].x; p.y = (*cloud)[pointIdxNKNSearch[1]].y; p.z = (*cloud)[pointIdxNKNSearch[1]].z;
				cloud_sequenced->points.push_back(p);
				previousPoint = currentPoint;
				currentPoint = p;
			}
			else
			{
				if (pointIdxNKNSearch.size() > 0)
				{
					for (std::size_t x = 0; x < pointIdxNKNSearch.size(); ++x)
					{
						PointXYZ p1;
						p1.x = (*cloud)[pointIdxNKNSearch[x]].x; p1.y = (*cloud)[pointIdxNKNSearch[x]].y; p1.z = (*cloud)[pointIdxNKNSearch[x]].z;
						if (!pointExistsInCLoud(p1, cloud_sequenced))
						{
							cloud_sequenced->points.push_back(p1);
							previousPoint = currentPoint;
							currentPoint = p1;
							break;
						}
					}
				}
			}
		}
	}

	cloud_sequenced->width = cloud_sequenced->points.size();
	cloud_sequenced->height = 1;
}

void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d& data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		Point& p = cloud->at(i);
		if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

void improveSurfaceBoundaryByFittingBSplineSurface(pcl::PointCloud<PointNormal>::Ptr meshPointCloud, pcl::PointCloud<PointNormal>::Ptr segmentedSurfacePointCloud, pcl::PointCloud<PointNormal>::Ptr improvedSurfacePointCloud)
{

	string line;
	double d1;
	double d2;
	double d3;
	stringstream ss;
	std::string str;
	std::string file_contents;
	int numberOfPoints;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_OriginalSurface(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Sampled(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*segmentedSurfacePointCloud, *cloud_OriginalSurface);
	numberOfPoints = cloud_OriginalSurface->points.size();


	pcl::RandomSample<pcl::PointXYZ> rs;
	rs.setInputCloud(cloud_OriginalSurface);
	rs.setSample(10000);

	rs.filter(*cloud_Sampled);


	pcl::io::savePCDFile("cloudForSpline.pcd", *cloud_Sampled);

	string pcd_file = "cloudForSpline.pcd";
	string file_3dm = "splineSurfaceOutput.3dm";


	// ############################################################################
	// load point cloud

	printf("  loading %s\n", pcd_file.c_str());
	pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
	pcl::PCLPointCloud2 cloud2;
	pcl::on_nurbs::NurbsDataSurface data;

	if (pcl::io::loadPCDFile(pcd_file, cloud2) == -1)
		throw std::runtime_error("  PCD file not found.");

	fromPCLPointCloud2(cloud2, *cloud);
	PointCloud2Vector3d(cloud, data.interior);
	printf("  %zu points in data set\n", cloud->size()); //sisung_change

	// ############################################################################
	// fit B-spline surface

	unsigned order(3);
	unsigned refinement(3);
	unsigned iterations(8);
	unsigned mesh_resolution(128);

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.4;
	params.boundary_weight = 0.0;

	// initialize
	printf("  surface fitting ...\n");
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);

	// mesh for visualization
	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);

	// surface refinement
	for (unsigned i = 0; i < refinement; i++)
	{
		fit.refine(0);
		fit.refine(1);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	}

	// surface fitting with final refinement level
	for (unsigned i = 0; i < iterations; i++)
	{
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	}

	// pcl::io::savePCDFile("bSpline_inital_pointcloud.pcd", *mesh_cloud);

	//viewer.close();
	pcl::PointCloud<pcl::PointNormal>::Ptr bSplineSurface(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr projectedPointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_EvaluatedPointsOnSurface(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr meshCloud_WithNormals(new pcl::PointCloud<pcl::PointNormal>);
	list<double> angles;
	list<double> dists;
	pcl::PointCloud<pcl::PointNormal>::Ptr improvedSurfacePointCloud_other(new pcl::PointCloud<pcl::PointNormal>);
	for (size_t i = 0; i < meshPointCloud->points.size(); i++)
	{
		if (meshPointCloud->points[i].x < -11 && meshPointCloud->points[i].x > -12)
		{
			if (meshPointCloud->points[i].y < 5 && meshPointCloud->points[i].y > 4)
			{
				if (meshPointCloud->points[i].z < 463 && meshPointCloud->points[i].z > 461)
				{
					cout << "Point found: " << meshPointCloud->points[i];
				}
			}
		}

		Eigen::Vector3d originalPt(meshPointCloud->points[i].x, meshPointCloud->points[i].y, meshPointCloud->points[i].z);
		Eigen::Vector2d hintPt(0.5, 0.5);
		// inverse mapping
		//Vector2d params;
		Eigen::Vector3d pt, tu, tv, n;
		double error;
		int im_max_steps = 100;
		double im_accuracy = 1e-5;
		Eigen::Vector2d paramsB = fit.inverseMapping(fit.m_nurbs, originalPt, hintPt, error, pt, tu, tv, im_max_steps, im_accuracy);

		double point[9];
		ON_3dVector normalEst;
		fit.m_nurbs.Evaluate(paramsB(0), paramsB(1), 1, 3, point);
		fit.m_nurbs.EvNormal(paramsB(0), paramsB(1), normalEst);

		pcl::PointXYZ ptOnFittedSurface;
		ptOnFittedSurface.x = point[0];
		ptOnFittedSurface.y = point[1];
		ptOnFittedSurface.z = point[2];

		pcl::PointXYZ ptOnOriginalMesh;
		ptOnOriginalMesh.x = meshPointCloud->points[i].x;
		ptOnOriginalMesh.y = meshPointCloud->points[i].y;
		ptOnOriginalMesh.z = meshPointCloud->points[i].z;

		pcl::Normal normOnFittedSurface;
		normOnFittedSurface.normal_x = -normalEst.x;
		normOnFittedSurface.normal_y = -normalEst.y;
		normOnFittedSurface.normal_z = -normalEst.z;

		pcl::Normal normOnOriginalMesh;
		normOnOriginalMesh.normal_x = meshPointCloud->points[i].normal_x;
		normOnOriginalMesh.normal_y = meshPointCloud->points[i].normal_y;
		normOnOriginalMesh.normal_z = meshPointCloud->points[i].normal_z;

		Eigen::Vector4f eigen_normal_pt1 = normOnFittedSurface.getNormalVector4fMap();
		Eigen::Vector4f eigen_normal_pt2 = normOnOriginalMesh.getNormalVector4fMap();
		double angleDiff = pcl::getAngle3D(eigen_normal_pt1, eigen_normal_pt2, true);

		angles.push_back(angleDiff);
		dists.push_back(pcl::euclideanDistance(ptOnFittedSurface, ptOnOriginalMesh));
		if (pcl::euclideanDistance(ptOnFittedSurface, ptOnOriginalMesh) < 3 && angleDiff > 140) // (angleDiff < 40)) // || angleDiff > 140)) //0.5) // 1 && (angleDiff < 30 || angleDiff > 150))
		{
			pcl::PointNormal ptTmp;
			ptTmp.x = ptOnOriginalMesh.x; // point[0];
			ptTmp.y = ptOnOriginalMesh.y; // point[1];
			ptTmp.z = ptOnOriginalMesh.z; // point[2];
			ptTmp.normal_x = meshPointCloud->points[i].normal_x;  //-normalEst.x;
			ptTmp.normal_y = meshPointCloud->points[i].normal_y; //-normalEst.y;
			ptTmp.normal_z = meshPointCloud->points[i].normal_z; //-normalEst.z;
			improvedSurfacePointCloud->points.push_back(ptTmp);
		}
		else if (pcl::euclideanDistance(ptOnFittedSurface, ptOnOriginalMesh) < 3 && angleDiff < 40) // || angleDiff > 140)) //0.5) // 1 && (angleDiff < 30 || angleDiff > 150))
		{
			pcl::PointNormal ptTmp;
			ptTmp.x = ptOnOriginalMesh.x; // point[0];
			ptTmp.y = ptOnOriginalMesh.y; // point[1];
			ptTmp.z = ptOnOriginalMesh.z; // point[2];
			ptTmp.normal_x = meshPointCloud->points[i].normal_x;  //-normalEst.x;
			ptTmp.normal_y = meshPointCloud->points[i].normal_y; //-normalEst.y;
			ptTmp.normal_z = meshPointCloud->points[i].normal_z; //-normalEst.z;
			improvedSurfacePointCloud_other->points.push_back(ptTmp);
		}

		pcl::PointNormal ptTmp_b;
		ptTmp_b.x = ptOnFittedSurface.x; // point[0];
		ptTmp_b.y = ptOnFittedSurface.y; // point[1];
		ptTmp_b.z = ptOnFittedSurface.z; // point[2];
		ptTmp_b.normal_x = -normalEst.x;  //-normalEst.x;
		ptTmp_b.normal_y = -normalEst.y; //-normalEst.y;
		ptTmp_b.normal_z = -normalEst.z; //-normalEst.z;
		bSplineSurface->points.push_back(ptTmp_b);


		pcl::PointNormal ptTmp;

		ptTmp.x = meshPointCloud->points[i].x; // point[0];
		ptTmp.y = meshPointCloud->points[i].y; // point[1];
		ptTmp.z = meshPointCloud->points[i].z; // point[2];
		ptTmp.normal_x = -normalEst.x;
		ptTmp.normal_y = -normalEst.y;
		ptTmp.normal_z = -normalEst.z;
		projectedPointCloudWithNormals->points.push_back(ptTmp);
	}

	if (improvedSurfacePointCloud_other->points.size() > improvedSurfacePointCloud->points.size())
	{
		pcl::copyPointCloud(*improvedSurfacePointCloud_other, *improvedSurfacePointCloud);
	}


	bSplineSurface->width = static_cast<int>(bSplineSurface->points.size());
	bSplineSurface->height = 1;
	// pcl::io::savePCDFile("bSplineSurfacePoints.pcd", *bSplineSurface);


	for (int i = 0; i < segmentedSurfacePointCloud->points.size(); i++)
	{
		pcl::PointNormal ptTmp;

		ptTmp.x = segmentedSurfacePointCloud->points[i].x; // point[0];
		ptTmp.y = segmentedSurfacePointCloud->points[i].y; // point[1];
		ptTmp.z = segmentedSurfacePointCloud->points[i].z; // point[2];
		ptTmp.normal_x = segmentedSurfacePointCloud->points[i].normal_x;
		ptTmp.normal_y = segmentedSurfacePointCloud->points[i].normal_y;
		ptTmp.normal_z = segmentedSurfacePointCloud->points[i].normal_z;
		improvedSurfacePointCloud->points.push_back(ptTmp);
	}

	improvedSurfacePointCloud->width = static_cast<int>(improvedSurfacePointCloud->points.size());
	improvedSurfacePointCloud->height = 1;

	//removing duplicate points
	pcl::PointCloud<PointNormal>::Ptr improvedSurfacePointCloudNoDuplicates(new pcl::PointCloud<PointNormal>);
	std::vector<PointNormal> vectorPointNormal;
	for (size_t i = 0; i < improvedSurfacePointCloud->points.size(); i++)
	{
		vectorPointNormal.push_back(improvedSurfacePointCloud->points[i]);
	}
	std::sort(vectorPointNormal.begin(), vectorPointNormal.end(), myobject2);

	if (vectorPointNormal.size() > 0)
	{
		for (size_t i = 0; i < vectorPointNormal.size() - 1; i++)
		{
			if ((vectorPointNormal[i].x == vectorPointNormal[i + 1].x) && (vectorPointNormal[i].y == vectorPointNormal[i + 1].y) && (vectorPointNormal[i].z == vectorPointNormal[i + 1].z))
			{

			}
			else
			{
				improvedSurfacePointCloudNoDuplicates->points.push_back(vectorPointNormal[i]);
			}
		}

		improvedSurfacePointCloudNoDuplicates->width = improvedSurfacePointCloudNoDuplicates->points.size();
		improvedSurfacePointCloudNoDuplicates->height = 1;
		// pcl::io::savePCDFile("improvedSurfacePointCloud.pcd", *improvedSurfacePointCloudNoDuplicates);
	}

	projectedPointCloudWithNormals->width = static_cast<int>(projectedPointCloudWithNormals->points.size());
	projectedPointCloudWithNormals->height = 1;
}

void getBreakLineForDecorativeParts(string fileName)
{
	int noOfClusters = getClusters_EuclideanDistBased(fileName);

	for (size_t t = 0; t < noOfClusters; t++)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithoutNormals(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

		pcl::io::loadPCDFile(fileName + "Cluster_" + to_string(t) + ".pcd", *cloud);

		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			PointXYZ p;
			p.x = cloud->points[i].x;
			p.y = cloud->points[i].y;
			p.z = cloud->points[i].z;

			pcl::Normal n;
			n.normal_x = cloud->points[i].normal_x;
			n.normal_y = cloud->points[i].normal_y;
			n.normal_z = cloud->points[i].normal_z;

			cloudWithoutNormals->points.push_back(p);
			normals->points.push_back(n);
		}
		cloudWithoutNormals->width = cloudWithoutNormals->points.size();
		cloudWithoutNormals->height = 1;
		normals->width = normals->points.size();
		normals->height = 1;


		//calculate boundary;
		pcl::PointCloud<pcl::Boundary> boundary;
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est;
		boundary_est.setInputCloud(cloudWithoutNormals);
		boundary_est.setInputNormals(normals);
		boundary_est.setRadiusSearch(3);
		//boundary_est.setAngleThreshold(PI/4);
		boundary_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		boundary_est.compute(boundary);


		//get points which on the boundary form point cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (boundary[i].boundary_point == 1)
			{
				boundaryCloud->points.push_back(cloudWithoutNormals->points[i]);
			}
		}
		boundaryCloud->width = boundaryCloud->points.size();
		boundaryCloud->height = 1;
		boundaryCloud->is_dense = true;

		// pcl::io::savePCDFile("boundary.pcd", *boundaryCloud);
		// pcl::io::loadPCDFile("boundary.pcd", *boundaryCloud);
		//pcl::copyPointCloud(*boundaryCloud, *cloud_BorderPointsCombined);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sequenced(new pcl::PointCloud<pcl::PointXYZ>);
		getPointsInSequence(boundaryCloud, cloud_sequenced);
		// pcl::io::savePLYFile(fileName + "_cluster" + to_string(t) + "_breakLine.ply", *cloud_sequenced);
	}
}

#include <omp.h>

const float DISTANCE_THRESHOLD = 1.0;
const float NORMAL_SIMILARITY_THRESHOLD = 0.95;
const float CURVATURE_THRESHOLD = 0.1;
const float NORMAL_SIMILARITY_THRESHOLD_verificiation = 0.75;
const float CURVATURE_THRESHOLD_verification = 0.25;

std::vector<int> getKNNPoints(const Eigen::Vector3f& center_point, const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, int K) {
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	pcl::PointNormal searchPoint;
	searchPoint.x = center_point(0);
	searchPoint.y = center_point(1);
	searchPoint.z = center_point(2);

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		return pointIdxNKNSearch;
	}
	else {
		return {};
	}
}

std::vector<std::pair<int, int>> findClosestPointPairsUsingKNN(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const pcl::PointIndices& cluster1, const pcl::PointIndices& cluster2, int K = 10) {
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<std::pair<int, int>> closestPairs;

	for (int idx1 : cluster1.indices) {
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		if (kdtree.nearestKSearch(cloud->points[idx1], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			for (int k = 0; k < K; ++k) {
				int idx2 = pointIdxNKNSearch[k];
				float distance = std::sqrt(pointNKNSquaredDistance[k]);
				if (distance < DISTANCE_THRESHOLD && std::find(cluster2.indices.begin(), cluster2.indices.end(), idx2) != cluster2.indices.end()) {
					closestPairs.emplace_back(idx1, idx2);
				}
			}
		}
	}
	return closestPairs;
}

Eigen::Vector3f calculateAverageNormalAroundPoint(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int pointIndex, int K = 30) {
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	Eigen::Vector3f averageNormal(0.0f, 0.0f, 0.0f);
	if (kdtree.nearestKSearch(cloud->points[pointIndex], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (int idx : pointIdxNKNSearch) {
			averageNormal += cloud->points[idx].getNormalVector3fMap();
		}
		averageNormal /= static_cast<float>(K);
		averageNormal.normalize();
	}
	return averageNormal;
}


template <typename T>
T clamp_v2(T v, T lo, T hi) {
	return std::max(lo, std::min(v, hi));
}

std::tuple<float, float, float, float, float, float> getTriangleAngles(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
	float p0p1 = (p0 - p1).norm();
	float p1p2 = (p1 - p2).norm();
	float p0p2 = (p0 - p2).norm();

	float a = (p0p1 * p0p1 + p1p2 * p1p2 - p0p2 * p0p2) / (2 * p0p1 * p1p2);
	float b = (p0p2 * p0p2 + p1p2 * p1p2 - p0p1 * p0p1) / (2 * p0p2 * p1p2);

	a = clamp_v2(a, -1.0f, 1.0f);
	b = clamp_v2(b, -1.0f, 1.0f);

	float angleP1 = std::acos(a) * 180.0f / M_PI;
	float angleP2 = std::acos(b) * 180.0f / M_PI;
	float angleP0 = 180.0f - angleP1 - angleP2;

	return { angleP0, angleP1, angleP2, p1p2, p0p2, p0p1 };
}

float getTriangleArea(float angleP0, float p1p2, float p0p2, float p0p1) {
	float R = p1p2 / (2 * std::sin((angleP0 / 180) * M_PI));
	if (angleP0 < 90) {
		return (std::sqrt(R * R - p0p1 * p0p1 / 4) * p0p1) / 4 + (std::sqrt(R * R - p0p2 * p0p2 / 4) * p0p1) / 4;
	}
	else {
		return ((p1p2 * p0p1 * p0p2) / (4 * R)) / 2;
	}
}

std::pair<float, float> getCurvature(const Eigen::Vector3f& center_point, const Eigen::Vector3f& center_normal, const std::vector<Eigen::Vector3f>& knn_points) {
	int k = knn_points.size();
	float KH = 0;
	float KG = 0;
	std::vector<float> Area(k);
	std::vector<float> angles_P0(k);
	std::vector<float> angles_P1(k);
	std::vector<float> angles_P2(k);

	for (int m = 0; m < k; ++m) {
		Eigen::Vector3f p1 = knn_points[m];
		Eigen::Vector3f p2 = knn_points[(m + 1) % k];

		float angleP0, angleP1, angleP2, p1p2, p0p2, p0p1;
		std::tie(angleP0, angleP1, angleP2, p1p2, p0p2, p0p1) = getTriangleAngles(center_point, p1, p2);

		angles_P0[m] = angleP0 * M_PI / 180;
		angles_P1[m] = angleP1 * M_PI / 180;
		angles_P2[m] = angleP2 * M_PI / 180;

		if (angleP0 == 0) {
			angleP0 = 0.000001;
		}

		Area[m] = getTriangleArea(angleP0, p1p2, p0p2, p0p1);
	}

	for (int m = 0; m < k; ++m) {
		float alpha = angles_P1[(m + k - 1) % k];
		float beta = angles_P2[m];
		Eigen::Vector3f P1P0 = knn_points[m] - center_point;
		KH += (1 / std::tan(alpha) + 1 / std::tan(beta)) * P1P0.dot(center_normal);
		KG += angles_P0[m];
	}

	float Am = std::accumulate(Area.begin(), Area.end(), 0.0f);
	KH = KH / (4 * Am);
	KG = (2 * M_PI - KG) / Am;

	return { KH, KG };
}

float calculateAverageCurvature(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const std::vector<int>& points) {
	float totalCurvature = 0.0f;
	for (int idx : points) {
		Eigen::Vector3f point = cloud->points[idx].getVector3fMap();
		Eigen::Vector3f normal = cloud->points[idx].getNormalVector3fMap();

		// Get K nearest neighbors
		std::vector<int> knnIndices = getKNNPoints(point, cloud, 30);
		std::vector<Eigen::Vector3f> knnPoints;
		for (int knnIdx : knnIndices) {
			knnPoints.push_back(cloud->points[knnIdx].getVector3fMap());
		}

		// Calculate curvature for the point
		std::pair<float, float> curvature = getCurvature(point, normal, knnPoints);
		totalCurvature += curvature.first;
	}
	return totalCurvature / points.size();
}

#include <pcl/common/pca.h>

Eigen::Vector3f calculatePrincipalAxis(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const std::vector<int>& points) {
	pcl::PointCloud<pcl::PointNormal>::Ptr selectedPointsCloud(new pcl::PointCloud<pcl::PointNormal>);
	for (int idx : points) {
		selectedPointsCloud->points.push_back(cloud->points[idx]);
	}

	pcl::PCA<pcl::PointNormal> pca;
	pca.setInputCloud(selectedPointsCloud);
	Eigen::Vector3f principalAxis = pca.getEigenVectors().col(0).head<3>();

	return principalAxis;
}

int selectInnerPointAlongLine(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, const pcl::PointIndices& cluster, const Eigen::Vector3f& linePoint, const Eigen::Vector3f& lineDirection, float minDistance) {
	std::vector<std::pair<int, float>> candidatePoints;

	for (int idx : cluster.indices) {
		Eigen::Vector3f point = cloud->points[idx].getVector3fMap();
		Eigen::Vector3f projection = linePoint + lineDirection * ((point - linePoint).dot(lineDirection) / lineDirection.squaredNorm());
		float distance = (point - projection).norm();
		if (distance > minDistance) {
			candidatePoints.emplace_back(idx, distance);
		}
	}

	if (candidatePoints.empty()) return -1;
	std::sort(candidatePoints.begin(), candidatePoints.end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
		return a.second < b.second;
		});

	return candidatePoints[candidatePoints.size() / 2].first;
}

bool verifyClusterMerge(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int innerPoint1, int innerPoint2, float normalThreshold, float curvatureThreshold) {
	Eigen::Vector3f normal1 = cloud->points[innerPoint1].getNormalVector3fMap();
	Eigen::Vector3f normal2 = cloud->points[innerPoint2].getNormalVector3fMap();
	float normalSimilarity = normal1.dot(normal2);

	if (normalSimilarity < normalThreshold) {
		return false;
	}

	float curvature1 = cloud->points[innerPoint1].curvature;
	float curvature2 = cloud->points[innerPoint2].curvature;
	float curvatureDifference = std::abs(curvature1 - curvature2);

	return (curvatureDifference < curvatureThreshold);
}


void mergeClusters(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::vector<pcl::PointIndices>& clusters) {

	std::vector<std::pair<int, int>> mergePairs;
	std::vector<bool> merged(clusters.size(), false);

	std::cout << "mergeClusters function called." << std::endl;

	// Step 1: Compare all cluster pairs and store merge information
	for (size_t i = 0; i < clusters.size(); ++i) {
		if (merged[i]) continue;

		for (size_t j = i + 1; j < clusters.size(); ++j) {
			if (merged[j]) continue;

			// Find closest point pairs using KNN
			std::vector<std::pair<int, int>> closestPairs = findClosestPointPairsUsingKNN(cloud, clusters[i], clusters[j]);
			if (closestPairs.size() < 3) continue;
			std::cout << "Condition 1 (closest point pairs) passed for clusters " << i << " and " << j << std::endl;

			// Extract boundary points
			std::vector<int> boundaryPoints1, boundaryPoints2;
			for (const auto& pair : closestPairs) {
				boundaryPoints1.push_back(pair.first);
				boundaryPoints2.push_back(pair.second);
			}

			// Check normal vector similarity for local patches
			bool normalsAreSimilar = false;
			for (const auto& pair : closestPairs) {
				Eigen::Vector3f avgNormal1 = calculateAverageNormalAroundPoint(cloud, pair.first);
				Eigen::Vector3f avgNormal2 = calculateAverageNormalAroundPoint(cloud, pair.second);
				float normalSimilarity = avgNormal1.dot(avgNormal2);
				if (normalSimilarity > NORMAL_SIMILARITY_THRESHOLD) {
					normalsAreSimilar = true;
					break;
				}
			}
			if (!normalsAreSimilar) {
				std::cout << "Condition 2 (normal vector similarity) failed for clusters " << i << " and " << j << std::endl;
				continue;
			}
			std::cout << "Condition 2 (normal vector similarity) passed for clusters " << i << " and " << j << std::endl;

			// Check boundary curvature similarity
			float avgCurvature1 = 0.0f;
			float avgCurvature2 = 0.0f;

			for (int idx : boundaryPoints1) {
				Eigen::Vector3f point = cloud->points[idx].getVector3fMap();
				Eigen::Vector3f normal = cloud->points[idx].getNormalVector3fMap();

				std::vector<int> knnIndices = getKNNPoints(point, cloud, 30);
				std::vector<Eigen::Vector3f> knnPoints;
				for (int knnIdx : knnIndices) {
					knnPoints.push_back(cloud->points[knnIdx].getVector3fMap());
				}

				std::pair<float, float> curvature = getCurvature(point, normal, knnPoints);
				avgCurvature1 += curvature.first;
			}
			avgCurvature1 /= boundaryPoints1.size();

			for (int idx : boundaryPoints2) {
				Eigen::Vector3f point = cloud->points[idx].getVector3fMap();
				Eigen::Vector3f normal = cloud->points[idx].getNormalVector3fMap();

				std::vector<int> knnIndices = getKNNPoints(point, cloud, 30);
				std::vector<Eigen::Vector3f> knnPoints;
				for (int knnIdx : knnIndices) {
					knnPoints.push_back(cloud->points[knnIdx].getVector3fMap());
				}

				std::pair<float, float> curvature = getCurvature(point, normal, knnPoints);
				avgCurvature2 += curvature.first;
			}
			avgCurvature2 /= boundaryPoints2.size();

			if (std::abs(avgCurvature1 - avgCurvature2) > CURVATURE_THRESHOLD) {
				std::cout << "Condition 3 (boundary curvature similarity) failed for clusters " << i << " and " << j << std::endl;
				continue;
			}
			std::cout << "Condition 3 (boundary curvature similarity) passed for clusters " << i << " and " << j << std::endl;

			// Verification before merging
			Eigen::Vector3f boundaryPoint1 = cloud->points[boundaryPoints1[0]].getVector3fMap();
			Eigen::Vector3f boundaryPoint2 = cloud->points[boundaryPoints2[0]].getVector3fMap();

			Eigen::Vector3f lineDirection1 = calculatePrincipalAxis(cloud, boundaryPoints1);
			Eigen::Vector3f lineDirection2 = calculatePrincipalAxis(cloud, boundaryPoints2);

			// float minDistance = 0.05; // 임의의 거리 값
			float minDistance = 2.5; // 임의의 거리 값

			// Declare innerPoint1 and innerPoint2 before use
			int innerPoint1 = selectInnerPointAlongLine(cloud, clusters[i], boundaryPoint1, lineDirection1, minDistance * 100);
			int innerPoint2 = selectInnerPointAlongLine(cloud, clusters[j], boundaryPoint2, lineDirection2, minDistance * 100);

			if (innerPoint1 == -1 || innerPoint2 == -1) {
				std::cout << "Verification failed (innerPoint selection) for clusters " << i << " and " << j << std::endl;
				continue;
			}

			if (!verifyClusterMerge(cloud, innerPoint1, innerPoint2, NORMAL_SIMILARITY_THRESHOLD_verificiation, CURVATURE_THRESHOLD_verification)) {
				std::cout << "Verification failed for clusters " << i << " and " << j << std::endl;
				continue;
			}
			std::cout << "Verification passed for clusters " << i << " and " << j << std::endl;

			// Add the pair to merge list
			mergePairs.emplace_back(i, j);
			merged[j] = true;  // Mark cluster j as merged
		}
	}

	// Step 2: Perform the actual merge
	for (const auto& mergePair : mergePairs) {
		int i = mergePair.first;
		int j = mergePair.second;

		clusters[i].indices.insert(clusters[i].indices.end(), clusters[j].indices.begin(), clusters[j].indices.end());
		clusters[j].indices.clear();  // Clear the merged cluster
	}

	// Remove empty clusters
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const pcl::PointIndices& cluster) {
		return cluster.indices.empty();
		}), clusters.end());
}

#include <random>

bool checkNormalsIntersection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::Normal>::Ptr normals1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::Normal>::Ptr normals2,
	double threshold = 0.2, double sample_fraction = 0.01)
{
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud2);

	int num_samples = 30;
	std::vector<int> sampled_indices;
	sampled_indices.reserve(num_samples);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, cloud1->points.size() - 1);

	for (int i = 0; i < num_samples; ++i) {
		sampled_indices.push_back(dis(gen));
	}

	std::cout << "Sampled indices: ";
	for (int i : sampled_indices) {
		std::cout << i << " ";
	}
	std::cout << std::endl;

	for (int i : sampled_indices)
	{
		const auto& pos = cloud1->points[i];
		const auto& norm = normals1->points[i];

		for (double t = 0; t <= 3; t += 0.05)
		{
			pcl::PointXYZ samplePoint;
			samplePoint.x = pos.x + t * norm.normal_x;
			samplePoint.y = pos.y + t * norm.normal_y;
			samplePoint.z = pos.z + t * norm.normal_z;

			std::vector<int> indices(1);
			std::vector<float> distances(1);
			if (tree.nearestKSearch(samplePoint, 1, indices, distances) > 0)
			{
				// std::cout << "Ray point: " << samplePoint << ", Distance: " << distances[0] << std::endl;
				if (distances[0] < threshold)
				{
					return true;
				}
			}
		}
	}

	return false;
}

void savePLYandXYZ(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud, const std::string& basePath)
{
	pcl::io::savePLYFile(basePath + ".ply", *cloud);

	Eigen::MatrixXd pointCloudMatrix(cloud->points.size(), 6);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		pointCloudMatrix.row(i) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
			cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z;
	}
	writeMatrix_to_XYZ(pointCloudMatrix, basePath + ".xyz", 6);
}

void convertPLYtoXYZ(string inputFile, string outFile)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPLYFile(inputFile, *mesh_cloud);
	Eigen::MatrixXd pointCloudMatrix(mesh_cloud->points.size(), 6);
	for (size_t i = 0; i < mesh_cloud->points.size(); i++)
	{
		pointCloudMatrix.row(i) << mesh_cloud->points[i].x, mesh_cloud->points[i].y, mesh_cloud->points[i].z, mesh_cloud->points[i].normal_x, mesh_cloud->points[i].normal_y, mesh_cloud->points[i].normal_z;
	}
	writeMatrix_to_XYZ(pointCloudMatrix, outFile, 6);
}

void surfaceSegmentation(std::string filePath, int minCluster, int noOfNeighbours, double smoothnessAngleThreshold, double curvatureThreshold)
{
    std::string tmp_path = filePath;
    int numberOfClustersCreated = -1;
    
    while (numberOfClustersCreated < 2)
    {
        if (numberOfClustersCreated != -1 && numberOfClustersCreated < 2)
        {
            std::cout << "Single cluster found...Increasing angle theshold (+2) and trying again..." << std::endl;
            smoothnessAngleThreshold -= 2;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);

        pcl::io::loadPLYFile(filePath, *cloudWithNormals);

        for (size_t i = 0; i < cloudWithNormals->points.size(); i++)
        {
            pcl::PointXYZ basicPoint;
            basicPoint.x = cloudWithNormals->points[i].x;
            basicPoint.y = cloudWithNormals->points[i].y;
            basicPoint.z = cloudWithNormals->points[i].z;
            cloud->points.push_back(basicPoint);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;

        pcl::PointCloud<pcl::PointNormal>::Ptr sampledPointCloudNormal(new pcl::PointCloud<pcl::PointNormal>);
        getNormalsOnSurface(cloud, sampledPointCloudNormal);
        for (size_t i = 0; i < sampledPointCloudNormal->points.size(); i++)
        {
            pcl::Normal normTemp;
            normTemp.normal_x = sampledPointCloudNormal->points[i].normal_x;
            normTemp.normal_y = sampledPointCloudNormal->points[i].normal_y;
            normTemp.normal_z = sampledPointCloudNormal->points[i].normal_z;
            normTemp.curvature = sampledPointCloudNormal->points[i].curvature;

            normals->points.push_back(normTemp);
        }

        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(minCluster);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(noOfNeighbours);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothnessAngleThreshold / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvatureThreshold);

        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        std::cout << "\nTotal points: " << cloud->points.size() << std::endl;
        std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

        std::sort(clusters.begin(), clusters.end(), myobject3);

        std::cout << "Points in " << std::endl;
        int totalPointsInClusters = 0;
        for (size_t i = 0; i < clusters.size(); i++)
        {
            std::cout << "Cluster " << std::to_string(i + 1) << " : " << clusters[i].indices.size() << std::endl;
            totalPointsInClusters = totalPointsInClusters + clusters[i].indices.size();
        }
        if (clusters.size() > 1)
        {
            std::cout << "Points in two clusters / Total Points in Pointcloud: " << std::setprecision(4) << static_cast<double>(100) * (clusters[0].indices.size() + clusters[1].indices.size()) / cloud->points.size() << "%" << std::endl;
            std::cout << "Points in two clusters / Total Points in all clusters: " << std::setprecision(4) << static_cast<double>(100) * (clusters[0].indices.size() + clusters[1].indices.size()) / totalPointsInClusters << "%" << std::endl;
            std::ofstream myfile("segmentationStats.txt", std::ofstream::out | std::ofstream::app);
            if (myfile.is_open())
            {
                myfile << fileNameOnly << "\t";
                myfile << static_cast<double>(100) * (clusters[0].indices.size() + clusters[1].indices.size()) / cloud->points.size() << "%\t" << static_cast<double>(100) * (clusters[0].indices.size() + clusters[1].indices.size()) / totalPointsInClusters << "%" << std::endl;
                myfile.close();
            }
        }

        // 병합 함수 호출
        std::cout << "Calling mergeClusters function." << std::endl;
        mergeClusters(cloudWithNormals, clusters);

        numberOfClustersCreated = clusters.size();

        if (clusters.size() > 1)
        {
            auto& largestCluster = clusters[0];
            auto& secondLargestCluster = clusters[1];

            // 가장 큰 두 클러스터의 포인트 및 노말 벡터 추출
            pcl::PointCloud<pcl::PointXYZ>::Ptr largestClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr largestClusterNormals(new pcl::PointCloud<pcl::Normal>);

            for (int idx : largestCluster.indices)
            {
                largestClusterCloud->points.push_back(cloud->points[idx]);
                largestClusterNormals->points.push_back(normals->points[idx]);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr secondLargestClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::Normal>::Ptr secondLargestClusterNormals(new pcl::PointCloud<pcl::Normal>);

            for (int idx : secondLargestCluster.indices)
            {
                secondLargestClusterCloud->points.push_back(cloud->points[idx]);
                secondLargestClusterNormals->points.push_back(normals->points[idx]);
            }

            // normal vector check

            // 노말 벡터 교차 여부 판단
            bool flipNormals = checkNormalsIntersection(largestClusterCloud, largestClusterNormals, secondLargestClusterCloud, secondLargestClusterNormals);

            // 전체 클러스터의 노말 벡터 반전
            if (flipNormals)
            {
                for (auto& normal : normals->points)
                {
                    normal.normal_x = -normal.normal_x;
                    normal.normal_y = -normal.normal_y;
                    normal.normal_z = -normal.normal_z;
                }

                // 노말 벡터가 반전되었음을 cloudWithNormals에 반영
                for (size_t i = 0; i < cloudWithNormals->points.size(); i++)
                {
                    cloudWithNormals->points[i].normal_x = normals->points[i].normal_x;
                    cloudWithNormals->points[i].normal_y = normals->points[i].normal_y;
                    cloudWithNormals->points[i].normal_z = normals->points[i].normal_z;
                }

                // 반전된 노말을 저장
                pcl::io::savePLYFile(tmp_path + "_flipped_normal.ply", *cloudWithNormals);
            }

            std::cout << std::boolalpha << "Normals flipped: " << flipNormals << std::endl;

            // Save clusters to files
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals_Cluster1(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals_Cluster2(new pcl::PointCloud<pcl::PointNormal>);
            for (size_t i = 0; i < clusters[0].indices.size(); i++)
            {
                cloudWithNormals_Cluster1->points.push_back(cloudWithNormals->points[clusters[0].indices[i]]);
            }
            cloudWithNormals_Cluster1->width = cloudWithNormals_Cluster1->points.size();
            cloudWithNormals_Cluster1->height = 1;

            for (size_t i = 0; i < clusters[1].indices.size(); i++)
            {
                cloudWithNormals_Cluster2->points.push_back(cloudWithNormals->points[clusters[1].indices[i]]);
            }
            cloudWithNormals_Cluster2->width = cloudWithNormals_Cluster2->points.size();
            cloudWithNormals_Cluster2->height = 1;

            // pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_0.ply", *cloudWithNormals_Cluster1);
            // pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_1.ply", *cloudWithNormals_Cluster2);

            pcl::PointCloud<pcl::PointNormal>::Ptr improvedSurfacePoints_Cluster1(new pcl::PointCloud<pcl::PointNormal>);
            improveSurfaceBoundaryByFittingBSplineSurface(sampledPointCloudNormal, cloudWithNormals_Cluster1, improvedSurfacePoints_Cluster1);
            cloudWithNormals_Cluster1->points.clear();
            pcl::copyPointCloud(*improvedSurfacePoints_Cluster1, *cloudWithNormals_Cluster1);

            pcl::PointCloud<pcl::PointNormal>::Ptr improvedSurfacePoints_Cluster2(new pcl::PointCloud<pcl::PointNormal>);
            improveSurfaceBoundaryByFittingBSplineSurface(sampledPointCloudNormal, cloudWithNormals_Cluster2, improvedSurfacePoints_Cluster2);
            cloudWithNormals_Cluster2->points.clear();
            pcl::copyPointCloud(*improvedSurfacePoints_Cluster2, *cloudWithNormals_Cluster2);

            // 다시 flipNormals가 true인 경우 노말을 반전
            if (flipNormals)
            {
                for (auto& normal : cloudWithNormals_Cluster1->points)
                {
                    normal.normal_x = -normal.normal_x;
                    normal.normal_y = -normal.normal_y;
                    normal.normal_z = -normal.normal_z;
                }
                for (auto& normal : cloudWithNormals_Cluster2->points)
                {
                    normal.normal_x = -normal.normal_x;
                    normal.normal_y = -normal.normal_y;
                    normal.normal_z = -normal.normal_z;
                }
            }

            pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_Improved_0.ply", *cloudWithNormals_Cluster1);
            pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_Improved_1.ply", *cloudWithNormals_Cluster2);

            // Getting file name without path and extension
            const size_t last_slash_idx = tmp_path.find_last_of("/");
            if (std::string::npos != last_slash_idx)
            {
                tmp_path.erase(0, last_slash_idx + 1);
            }
            const size_t period_idx = tmp_path.rfind('.');
            if (std::string::npos != period_idx)
            {
                tmp_path.erase(period_idx);
            }

            pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_0.ply", *cloudWithNormals_Cluster1);
            pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_1.ply", *cloudWithNormals_Cluster2);

            uniformSampling(tmp_path + "tmpSurfaceCluster_0.ply", tmp_path + "_UniformSampled_0.ply");
            uniformSampling(tmp_path + "tmpSurfaceCluster_1.ply", tmp_path + "_UniformSampled_1.ply");

            // PLY 파일을 XYZ 파일로 변환하여 저장하는 코드 추가
            //convertPLYtoXYZ(tmp_path + "_UniformSampled_0.ply", tmp_path + "_Surface_0.xyz");
            //convertPLYtoXYZ(tmp_path + "_UniformSampled_1.ply", tmp_path + "_Surface_1.xyz");

            for (size_t t = 0; t < clusters.size(); t++)
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals_Cluster(new pcl::PointCloud<pcl::PointNormal>);
                for (size_t i = 0; i < clusters[t].indices.size(); i++)
                {
                    cloudWithNormals_Cluster->points.push_back(cloudWithNormals->points[clusters[t].indices[i]]);
                }
                cloudWithNormals_Cluster->width = cloudWithNormals_Cluster->points.size();
                cloudWithNormals_Cluster->height = 1;
                // pcl::io::savePLYFile("cluster_" + std::to_string(t) + "0.ply", *cloudWithNormals_Cluster);
            }
        }
        else if (clusters.size() == 1)
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals_Cluster1(new pcl::PointCloud<pcl::PointNormal>);
            for (size_t i = 0; i < clusters[0].indices.size(); i++)
            {
                cloudWithNormals_Cluster1->points.push_back(cloudWithNormals->points[clusters[0].indices[i]]);
            }
            cloudWithNormals_Cluster1->width = cloudWithNormals_Cluster1->points.size();
            cloudWithNormals_Cluster1->height = 1;

            // Getting file name without path and extension
            const size_t last_slash_idx = tmp_path.find_last_of("/");
            if (std::string::npos != last_slash_idx)
            {
                tmp_path.erase(0, last_slash_idx + 1);
            }
            const size_t period_idx = tmp_path.rfind('.');
            if (std::string::npos != period_idx)
            {
                tmp_path.erase(period_idx);
            }

            // pcl::io::savePLYFile(tmp_path + "tmpSurfaceCluster_0.ply", *cloudWithNormals_Cluster1);
            break;
        }

        //------------------------------------------------------
        // Working with unclustered points
        //------------------------------------------------------
        // Getting unclustered points in separate cloud

        std::list<int> allPointsInClusteres_indices;
        for (size_t t = 0; t < clusters.size(); t++)
        {
            for (size_t p = 0; p < clusters[t].indices.size(); p++)
            {
                allPointsInClusteres_indices.push_back(clusters[t].indices[p]);
            }
        }
        allPointsInClusteres_indices.sort();
        pcl::PointCloud<pcl::PointNormal>::Ptr points_unclustered(new pcl::PointCloud<pcl::PointNormal>);
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            bool found = (std::find(allPointsInClusteres_indices.begin(), allPointsInClusteres_indices.end(), i) != allPointsInClusteres_indices.end());
            if (!found)
            {
                points_unclustered->points.push_back(cloudWithNormals->points[i]);
            }
        }
        points_unclustered->width = points_unclustered->points.size();
        points_unclustered->height = 1;
        pcl::io::savePLYFile(outPath + fileNameOnly + "_unclustered.ply", *points_unclustered);
        getBreakLineForDecorativeParts(outPath + fileNameOnly + "_unclustered.ply");

        numberOfClustersCreated = clusters.size();
    }
}

std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> rgb(double minimum, double maximum, double value)
{
	double ratio = 2 * (value - minimum) / (maximum - minimum);
	std::uint8_t b = int(std::max(static_cast<double>(0), 255 * (1 - ratio)));
	std::uint8_t r = int(std::max(static_cast<double>(0), 255 * (ratio - 1)));
	std::uint8_t g = 255 - b - r;
	return std::make_tuple(r, g, b);
}

void getClusters(string fileName)
{
	pcl::PointCloud<PointTypeIO>::Ptr cloud_in(new pcl::PointCloud<PointTypeIO>), cloud_out(new pcl::PointCloud<PointTypeIO>);
	pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
	pcl::search::KdTree<PointTypeIO>::Ptr search_tree(new pcl::search::KdTree<PointTypeIO>);
	pcl::console::TicToc tt;
	pcl::PointCloud<pcl::PointNormal>::Ptr missedPoints(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PCDWriter writer;
	pcl::io::loadPLYFile(fileName, *missedPoints);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(missedPoints);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
	ec.setClusterTolerance(3); // 2cm
	ec.setMinClusterSize(100); // 100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(missedPoints);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointNormal>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->push_back((*missedPoints)[*pit]); //*
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
		std::stringstream ss;
		ss << fileName + "Cluster_" << j << ".pcd";
		writer.write<pcl::PointNormal>(ss.str(), *cloud_cluster, false); //*
		j++;
	}
}

namespace fs = std::experimental::filesystem;

int main(int argc, char** argv) {
    std::string path = "../Dataset/Point"; 
    std::string outputBasePath = "../Output/"; 
    std::string tempPath = outputBasePath + "Temp/"; 
    
    if (!fs::exists(outputBasePath)) {
        fs::create_directories(outputBasePath);
    }
    if (!fs::exists(tempPath)) {
        fs::create_directories(tempPath);
    }
    
    Timer t;
    Timer t_individual;

    std::vector<fs::path> pcd_files;
    for (const auto& entry : fs::recursive_directory_iterator(path)) {
        if (!fs::is_directory(entry.path()) && entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path());
        }
    }
    std::sort(pcd_files.begin(), pcd_files.end());

    ofstream myfile(outputBasePath + "segmentationTimes.txt", std::ofstream::out | std::ofstream::app);

    for (const auto& file_path : pcd_files) {
        std::string inputFilePath = file_path.string();
        std::string fileNameOnly = file_path.stem().string().substr(0, 14);
        
        std::string currentOutputDir = outputBasePath + fileNameOnly + "/";
        if (!fs::exists(currentOutputDir)) {
            fs::create_directories(currentOutputDir);
        }

        std::cout << "Starting processing on: " << inputFilePath << std::endl;
        
        setInputFile(inputFilePath, tempPath);
        std::string downsampledFilePath = downsamplePointCloud(inputFilePath);

        myfile.open(outputBasePath + "segmentationTimes.txt", std::ofstream::out | std::ofstream::app);
        if (myfile.is_open()) {
            myfile << fileNameOnly << "\t" << t_individual.elapsed() << "\t";
            myfile.close();
        }
        t_individual.reset();

        int minCluster = 50, noOfNeighbours = 10;
        double smoothnessAngleThreshold = 4.5, curvatureThreshold = 1.5;
        std::cout << "Starting surface segmentation on: " << downsampledFilePath << std::endl;
        surfaceSegmentation(downsampledFilePath, minCluster, noOfNeighbours, smoothnessAngleThreshold, curvatureThreshold);

        std::string tempFileName0 = tempPath + fileNameOnly + "_SampledWithNormals.plytmpSurfaceCluster_Improved_0.ply";
        std::string tempFileName1 = tempPath + fileNameOnly + "_SampledWithNormals.plytmpSurfaceCluster_Improved_1.ply";

        std::string outputFileName0 = currentOutputDir + fileNameOnly + "_Surface_0.ply";
        std::string outputFileName1 = currentOutputDir + fileNameOnly + "_Surface_1.ply";

        std::cout << "Ply to XYZ start" << std::endl;

        if (fs::exists(tempFileName0)) {
            fs::copy_file(tempFileName0, outputFileName0, fs::copy_options::overwrite_existing);
            convertPLYtoXYZ(outputFileName0, currentOutputDir + fileNameOnly + "_Surface_0.xyz");
        } else {
            std::cout << "Warning: Source file not found: " << tempFileName0 << std::endl;
        }

        if (fs::exists(tempFileName1)) {
            fs::copy_file(tempFileName1, outputFileName1, fs::copy_options::overwrite_existing);
            convertPLYtoXYZ(outputFileName1, currentOutputDir + fileNameOnly + "_Surface_1.xyz");
        } else {
            std::cout << "Warning: Source file not found: " << tempFileName1 << std::endl;
        }

        myfile.open(outputBasePath + "segmentationTimes.txt", std::ofstream::out | std::ofstream::app);
        if (myfile.is_open()) {
            cout << "\nTime after processing: " << t_individual.elapsed() << " seconds" << endl;
            myfile << t_individual.elapsed() << endl;
            myfile.close();
        }
        t_individual.reset();
    }

    myfile.open(outputBasePath + "segmentationTimes.txt", std::ofstream::out | std::ofstream::app);
    if (myfile.is_open()) {
        myfile << "\nTotal time elapsed: " << t.elapsed() << " seconds\n";
        myfile.close();
    }

    std::cout << "Total time elapsed: " << t.elapsed() << " seconds" << std::endl;

    return EXIT_SUCCESS;
}