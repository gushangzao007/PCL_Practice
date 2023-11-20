#include <iostream>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>

int main(int argc, char **argv) {

    float LenX;
    float LenY;
    float LenZ;
    float SmoothRadius;
    float MaxEdge;
    float Mu;
    float MaxNeighbors;
    float MaxSurfaceAngle;



    // 提示用户输入整数
    std::cout << "LenX（0.005）";
    std::cin >> LenX;
    std::cout << "LenY（0.03）";
    std::cin >> LenY;
    std::cout << "LenZ（1）";
    std::cin >> LenZ;
    std::cout << "SmoothRadius（0.6）";
    std::cin >> SmoothRadius;

    std::cout << "最大三角形边长(1)";
    std::cin >> MaxEdge;
    std::cout << "边长乘数(10000)";
    std::cin >> Mu;
    std::cout << "最大搜索点数（25）";
    std::cin >> MaxNeighbors;

    std::cout << "MaxSurfaceAngle（45 deg）";
    std::cin >> MaxSurfaceAngle;

    //1. read ply file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file.\n");
        return -1;
    }

    

    //2. grid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;  // Create the filtering object
    sor.setInputCloud (cloud);
    sor.setLeafSize (LenX, LenY, LenZ);
    sor.filter (*cloud_filtered);

    pcl::visualization::PCLVisualizer viewer1("grid filtered Viewer");
    viewer1.addPointCloud(cloud_filtered, "original cloud");
    viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original cloud");
    viewer1.spin();

    //3. smooth
        // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr  mls_points(new pcl::PointCloud<pcl::PointNormal>);
        // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    // Set parameters
    mls.setInputCloud (cloud_filtered);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (SmoothRadius);
        // Reconstruct
    mls.process (*mls_points);

    pcl::visualization::PCLVisualizer viewer2("Mesh Viewer");
    viewer2.addPointCloud<pcl::PointNormal>(mls_points,"original cloud");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original cloud");
    viewer2.spin();

        // Save output
    //pcl::io::savePCDFile ("out3dmPLY2-elm.pcd", *mls_points);

    // //X.可视化原始点云和重建后的平滑点云
    // pcl::visualization::PCLVisualizer viewer("PLY Viewer");
    // viewer.addPointCloud(cloud_filtered, "original cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original cloud");
    // viewer.spin();

    //3. get Mesh
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(mls_points);
        // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
        // Set the maximum distance between connected points (maximum edge length)  0.025
    gp3.setSearchRadius (MaxEdge);
        // Set typical values for the parameters  2.5
    gp3.setMu (Mu);
    gp3.setMaximumNearestNeighbors (MaxNeighbors);  //100
    gp3.setMaximumSurfaceAngle(M_PI*MaxSurfaceAngle/180); // 45 degrees
    gp3.setMinimumAngle(0);//(M_PI/36); // 10 degrees  /18
    gp3.setMaximumAngle(M_PI);//(2*M_PI/2.5); // 120 degrees  /3
    gp3.setNormalConsistency(true);
        // Get result
    gp3.setInputCloud (mls_points);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
        // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    pcl::visualization::PCLVisualizer viewer("Mesh Viewer");
    viewer.addPolygonMesh(triangles, "mesh");
    viewer.spin();



    return 0;

}