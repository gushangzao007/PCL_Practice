#include <iostream>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv) {

    float LenX;
    float LenY;
    float LenZ;

    std::cout << "LenX（0.005）";
    std::cin >> LenX;
    std::cout << "LenY（0.03）";
    std::cin >> LenY;
    std::cout << "LenZ（1）";
    std::cin >> LenZ;

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
   

    //可视化原始点云和重建后的平滑点云
    pcl::visualization::PCLVisualizer viewer("PLY Viewer");
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    //viewer.addCoordinateSystem(1.0);
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer.addPointCloud(cloud_filtered, "filtered cloud");
    //viewer.addCoordinateSystem(1, "cloud", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered cloud");
    viewer.spin();


    return 0;
}