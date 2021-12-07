/**
 * @author David Tapiador de Vera
**/

#include <iostream>
#include <thread>
#include <stdlib.h>
#include <time.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>






using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Practise 6"));
int pc_adddeds = 0;

void printUsage (const char* progName)
{
  std::cout << "\nUsage: "<<progName<<" filename.pcd\n\n";
}

void add_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	std::string string = "cloud" + std::to_string(pc_adddeds);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, string);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, string);
	pc_adddeds++;
}

pcl::visualization::PCLVisualizer::Ptr visualize ()
{
	/////////////////////////////////////
	// Open 3D viewer and add point cloud
	/////////////////////////////////////
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}

int main(int argc, char** argv)
{
	int input_veces;
	std::string filename;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::PCLVisualizer::Ptr viewer;
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	srand(time(NULL));
	//----------------------------------------------------
	// Parse Command Line Arguments: file .pcd
	// and Read file .pcd
	//----------------------------------------------------
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
	if (pcd_filename_indices.empty ()) {
		printUsage (argv[0]);
		return 0;
	}
	else {
		// Read file .pcd
		filename = argv[pcd_filename_indices[0]];
		if (pcl::io::loadPCDFile (filename, *point_cloud_ptr) == -1) {
			std::cout << "Was not able to open file \""<<filename<<"\".\n";
			printUsage (argv[0]);
			return 0;
		}
		reader.read (filename, *cloud_blob);
		//Input from user
		std::cout << "Type a number -> "; 
		std::cin >> input_veces;
	}

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	writer.write<pcl::PointXYZ> ("plane_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	// Create the segmentation object
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);

	// Create the filtering object
	int nr_points = (int) cloud_filtered->size ();


	// While 30% of the original cloud is still there
	for ( int i = 0; i < input_veces; i++ )
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		std::stringstream ss;
		ss << "plane" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);

	}
	
	// Show the filtered image (the image without the biggest planes)
	add_cloud(cloud_filtered);

	viewer = visualize();
	while (!viewer->wasStopped ()) {
		viewer->spinOnce (100);
		std::this_thread::sleep_for(100ms);
	}

	return (0);
}