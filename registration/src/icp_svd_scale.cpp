#include <iostream>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <boost/random.hpp>

#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

using namespace std;

enum methods {
  SVD,
  DQ,
  LM
};

void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char** argv)
{
  
  int method = SVD;
  pcl::console::parse_argument (argc, argv, "-m", method);
  std::cout << "method: ";
  switch (method) {
  case SVD: std::cout << "SVD"; break;
  case DQ:  std::cout << "DQ";  break;
  case LM:  std::cout << "LM";  break;
  default: std::cout << "undefined. ERROR" << std::endl; exit(0);
  }
  std::cout << std::endl;

  bool use_scale = false;
  pcl::console::parse_argument (argc, argv, "-s", use_scale);
  std::cout << "use scale: " << (use_scale ? "true" : "false") << std::endl;
  if (use_scale) {
	std::cout << "forse SVD." << std::endl;
	method = SVD;
  }

  bool use_rand = false;
  pcl::console::parse_argument (argc, argv, "-r", use_rand);
  std::cout << "use random seed: " << (use_rand ? "true" : "false") << std::endl;

  
  
  boost::random::mt19937 gen; // alternative to rand()
  if (use_rand)
    gen.seed( std::time(0) ); // random seed with current time in second
  else
    gen.seed( 0 ); // fixed seed

  // 一様実数分布
  boost::random::uniform_real_distribution<float> frand( 1.0, 3.2 ); // random gen between 1.0 and 3.2
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  // create random source point cloud
  for (int i = 0; i < 1000; i++) {
    cloud_source->push_back (pcl::PointXYZ (frand(gen), frand(gen), frand(gen) ));
  }
  

  // create random transformation: R and T
  Eigen::Affine3f transformation_true;
  {
    // random rotation matrix
    Eigen::Vector3f axis;
    axis.setRandom().normalize();// random:[-1:1]
    float angle = frand( gen );
    // 軸と角度から回転行列を求める
    Eigen::Affine3f R ( Eigen::AngleAxis<float> ( angle, axis ) );
    
    // random translation vector
    Eigen::Translation3f T ( frand(gen), frand(gen), frand(gen) );

    std::cout << "true R" << std::endl << R.matrix() << std::endl
              << "true T" << std::endl << T .vector() << std::endl;

    if ( use_scale )
	  {
		float scale = frand( gen );
		R.matrix().topLeftCorner(3,3) *= scale;// 3x3すべてscaleする
		std::cout << "true sR" << std::endl << R.matrix() << std::endl
				  << "true scale " << scale << std::endl;
	  }
    
    // R and T
    transformation_true = T * R ; // shoul be in this order if you mean (Rx + T).   If R*T, then R(x+t) !

  }
  std::cout << "true transformation" << std::endl << transformation_true.matrix() << std::endl;

  
  // create target point cloud
  pcl::transformPointCloud ( *cloud_source, *cloud_target, transformation_true );
    
  boost::shared_ptr< pcl::registration::TransformationEstimation< pcl::PointXYZ, pcl::PointXYZ > > estPtr;
  if ( use_scale )
    // estimator of R and T along with scale
    estPtr.reset ( new pcl::registration::TransformationEstimationSVDScale < pcl::PointXYZ, pcl::PointXYZ > () );
  else 
    // estimator of R and T
    switch (method) {
    case SVD:
      estPtr.reset ( new pcl::registration::TransformationEstimationSVD < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case DQ:
      estPtr.reset ( new pcl::registration::TransformationEstimationDualQuaternion < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    case LM:
      estPtr.reset ( new pcl::registration::TransformationEstimationLM < pcl::PointXYZ, pcl::PointXYZ > () );
      break;
    }
    
  Eigen::Affine3f transformation_est;
  estPtr->estimateRigidTransformation ( *cloud_source,
                                        *cloud_target,
                                        transformation_est.matrix() );
  
  if ( use_scale ) {
    Eigen::Matrix3f R = transformation_est.matrix().topLeftCorner(3,3);
    std::cout << "estimated scale " << std::sqrt( (R.transpose() * R).trace() / 3.0 ) << std::endl;
	cout << "scale.x: " << R.row(0).norm() << endl
		 << "      y: " << R.row(1).norm() << endl
		 << "      z: " << R.row(2).norm() << endl;
  }
  std::cout << "estimated transformation " << std::endl << transformation_est.matrix()  << std::endl;

  // viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_source (cloud_source, 100, 155, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, color_source, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_target (cloud_target, 155, 150, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_target, color_target, "target");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_trans (cloud_target, 255, 5, 30);
  cloud_source_trans = cloud_source;
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source_trans, color_trans, "trans");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "trans");

  
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(-6.22997, -7.03095, -24.7261, -0.163784, 0.958968, -0.231419);
  
  viewer->spinOnce();
  boost::this_thread::sleep (boost::posix_time::microseconds (1000000));

  //IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp ( new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> () );
  icp->setTransformationEstimation(estPtr);// default:point-to-point
  icp->setInputSource(cloud_source);
  icp->setInputTarget(cloud_target);
  icp->setMaximumIterations(50);
  icp->align ( *cloud_source_trans );
  if(icp->hasConverged()){
	transformation_est.matrix() = icp->getFinalTransformation ().cast<float>();
	print4x4Matrix (transformation_est.matrix());
  }

  viewer->updatePointCloud ( cloud_source_trans, color_trans, "trans" );
  viewer->spin();
  boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
  
  int iterations(1);
  while ( !viewer->wasStopped() )
	{
	  // registration
	  icp->align ( *cloud_source_trans );
    
	  if ( icp->hasConverged() )
		{
		  viewer->updatePointCloud ( cloud_source_trans, color_trans, "trans" );
		  //std::cout << icp->getFitnessScore() << std::endl;
		  std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
		  transformation_est.matrix() *= icp->getFinalTransformation ().cast<float>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
		  print4x4Matrix (transformation_est.matrix());  // Print the transformation between original pose and current pose

		  //ss.str ("");
		  //ss << iterations;
		  //std::string iterations_cnt = "ICP iterations = " + ss.str ();
		  //viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
		  //viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
		}
	  else
		std::cout << "Not converged." << std::endl;
    
	  viewer->spinOnce();
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


  return ( 0 );
}
