/* Remove sparse outliers from noisy data, using StatisticalRemoval
   All input and output files must be in .pcd format
   Input  : point cloud  "filename" 
   Output : point cloud of inliers  "filename_inliers" 
            point cloud of outliers "filename_outliers" 
*/

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>



//////////////////////////////////
// Help
//////////////////////////////////
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" filename.pcd\n\n";
}



//////////////////////////////////
// Main
//////////////////////////////////
int main (int argc, char** argv)
{
   std::string filename;

   pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);

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
   }



   //----------------------------------------------------
   // StatisticalOutlierRemoval Filter
   //----------------------------------------------------
   // Create the filtering object
   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> f;
   // pass the input point cloud to the processing module
   f.setInputCloud (point_cloud_ptr);
   // set some parameters 
   f.setMeanK (50); 
   f.setStddevMulThresh (1.0);
   // get the output point cloud
   f.filter (*cloud_filtered_ptr);
   

   //----------------------------------------------------
   // Write inliers point cloud in format file .pcd
   //----------------------------------------------------
   // remove file name extension
   filename = filename.substr(0, filename.find_last_of("."));

   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZ> (filename+"_inliers.pcd", *cloud_filtered_ptr, false);


   //----------------------------------------------------
   // Compute outliers
   //----------------------------------------------------
   f.setNegative(true);
   f.filter(*cloud_filtered_ptr);

   //----------------------------------------------------
   // Write outliers point cloud in format file .pcd
   //----------------------------------------------------
   writer.write<pcl::PointXYZ> (filename+"_outliers.pcd", *cloud_filtered_ptr, false);
   
}