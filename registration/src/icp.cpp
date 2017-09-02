#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
	{
	  cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	  cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
			<< std::endl;
  for (size_t i = 0; i < cloud_in->points.size (); ++i){
	std::cout << "    " << cloud_in->points[i].x
			  << " " << cloud_in->points[i].y
			  << " " << cloud_in->points[i].z
			  << std::endl;
  }
  
  *cloud_out = *cloud_in;
  
  std::cout << "size:" << cloud_out->points.size() << std::endl;

  // x方向に0.7ずらす
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
			<< std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

  // ICP Algorithm
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  //変換matrixを表示する
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  transformation_matrix = icp.getFinalTransformation ().cast<double>();
  print4x4Matrix (transformation_matrix);

  // 表示
  int vp[2];
  pcl::visualization::PCLVisualizer viewer("Viewer");
  //pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (cloud_in, 0, 255, 0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp[0]);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, vp[1]);
  viewer.addCoordinateSystem(1.0, "cloud_in",vp[0]);
  viewer.addCoordinateSystem(1.0, "cloud_out",vp[1]);
  viewer.addPointCloud(cloud_in,single_color, "cloud_in", vp[0]);
  viewer.addPointCloud(cloud_out, "cloud_out", vp[1]);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud_in", vp[0]);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloud_out", vp[1]);

  
  viewer.addText("original cloud", 20, 20, 30, 10,10,10, "input", vp[0]);
  viewer.addText("transformed cloud", 20, 20, 30, 1,1,1,"output", vp[1]);
  viewer.spin();
  
  return (0);
}

