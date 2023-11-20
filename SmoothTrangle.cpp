#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

int
main ()
{

  float MaxEdge;
  float Mu;
  float MaxNeighbors;
  float Radius;

    std::cout << "平滑搜索半径(0.2)";
    std::cin >> Radius;
    // prompt 
    std::cout << "最大三角形边长(0.025)";
    std::cin >> MaxEdge;

    std::cout << "边长乘数(2.5)";
    std::cin >> Mu;

    std::cout << "最大搜索点数(100)";
    std::cin >> MaxNeighbors;



  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  //1. read ply file
  pcl::io::loadPLYFile("out3dmPLY2.ply", *cloud);


  // MovingLeastSquares
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (Radius);
  // Reconstruct
  mls.process (mls_points);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> mls_points_ptr = std::make_shared<pcl::PointCloud<pcl::PointNormal>>(mls_points);
  tree2->setInputCloud(mls_points_ptr);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)  0.025
  gp3.setSearchRadius (MaxEdge);

  // Set typical values for the parameters  2.5
  gp3.setMu (Mu);
  gp3.setMaximumNearestNeighbors (MaxNeighbors);  //100
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (mls_points_ptr);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
  viewer.addPolygonMesh(triangles, "mesh");

    // 启动可视化循环
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

  // Finish
  return (0);
}