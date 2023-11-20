#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

int
main ()
{
  float Radius;

    // 提示用户输入整数
    std::cout << "请输入搜索半径(0.2)";
    std::cin >> Radius;


  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  //1. read ply file
  pcl::io::loadPLYFile("out3dmPLY2.ply", *cloud);
        

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (Radius);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("out3dmPLY2-elm.pcd", mls_points);

}