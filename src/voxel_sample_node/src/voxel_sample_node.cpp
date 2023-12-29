#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/octree/octree_search.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr subFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                                               pcl::IndicesPtr indices,
                                               float cell_x,
                                               float cell_y,
                                               float cell_z)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pointcloud);
    sor.setIndices(indices);
    sor.setLeafSize(cell_x, cell_y, cell_z);
    sor.filter(*filtered_pointcloud); // No problem :)
    return filtered_pointcloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OctFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                                               float cell_x,
                                               float cell_y,
                                               float cell_z)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(2); // // Octree resolution - side length of octree voxels
    octree.setInputCloud(cloudIn);
    octree.addPointsFromInputCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it)
    {

        pcl::IndicesPtr indexVector(new std::vector<int>);
        pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();

        container.getPointIndices(*indexVector);
        *filtered_cloud += *subFilter(cloudIn, indexVector, cell_x, cell_y, cell_z);
    }
    return filtered_cloud;
}

// 接收到订阅的消息后，会进入消息回调函数
void voxelCallback(const sensor_msgs::PointCloud2ConstPtr &lasermsg)
{
    // 接收点云消息
    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sample_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*lasermsg, *combined_points); // ros转pcl点云

    //保存到PCD文件

    // 采用体素降采样处理
    for (int i = 0; i < 5; i++)
    {
        time_t task0 = clock();
        float downsize = 0.1 + i * 0.1;

        pcl::VoxelGrid<pcl::PointXYZI> downsample_filter;            // 创建滤波对象
        downsample_filter.setLeafSize(downsize, downsize, downsize); // 设置滤波时创建的体素体积为1cm的立方体
        downsample_filter.setDownsampleAllData(true);                // 对全字段进行下采样;
        downsample_filter.setInputCloud(combined_points);            // 设置需要过滤的点云给滤波对象
        downsample_filter.filter(*sample_points);                    // 执行滤波处理，存储输出
        time_t task1 = clock();

        printf("lidar pointclouds size is %ld\n", combined_points->points.size());
        printf("sample pointclouds size is %ld\n", sample_points->points.size());
        printf("the downsize is %f, time is %f!\n", downsize, double(task1 - task0) / 1000);
    }

    // 大量点云进行体素降采样

    time_t task2 = clock();
    float downsize = 0.1;
    sample_points = OctFilter(combined_points, downsize, downsize, downsize);
    time_t task3 = clock();
    printf("lidar pointclouds size is %ld\n", combined_points->points.size());
    printf("sample pointclouds size is %ld\n", sample_points->points.size());
    printf("pcl OctFilter time is %f!\n",double(task3 - task2) / 1000);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "voxel_sample_node");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber lidar_sub = n.subscribe("/lidar_pipline/sample_pointclouds", 10, voxelCallback);

    pcl::PointCloud<pcl::PointXYZI>::Ptr subFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud,
                                                   pcl::IndicesPtr indices,
                                                   float cell_x,
                                                   float cell_y,
                                                   float cell_z);
    pcl::PointCloud<pcl::PointXYZI>::Ptr OctFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                                                   float cell_x,
                                                   float cell_y,
                                                   float cell_z);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
