#include <pcl/range_image/range_image.h> //深度图像的头文件
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "preprocess.h"
// 这个函数设置视角

// void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
// {
//     Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
//     Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
//     Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
//     viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
//                              look_at_vector[0], look_at_vector[1], look_at_vector[2],
//                              up_vector[0], up_vector[1], up_vector[2]);
//     // 前三个参数为相机的位置，中间三个参数为看向哪个坐标，最后三个参数为以哪个坐标轴为上方
// }

int main(int argc, char **argv)
{

    // 初始化ROS节点
    ros::init(argc, argv, "voxel_sample_node");

    // 创建节点句柄
    ros::NodeHandle n;
    ros::Publisher pub_text_marker;
    pub_text_marker = n.advertise<sensor_msgs::PointCloud2>("/lidar_process/filter_car_pointclouds", 10);

    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    for (float y = -0.5f; y <= 0.5f; y += 0.01f)
    {
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = 1;
            pointCloud.points.push_back(point);
        }
    }
    pointCloud.width = (uint32_t)pointCloud.points.size();
    pointCloud.height = 1;
    printf("init pointclouds!");

    sensor_msgs::PointCloud2 filter_car_pointclouds_msg;   // 发布预处理点云
    pcl::toROSMsg(pointCloud, filter_car_pointclouds_msg); // 点云转换为msg
    filter_car_pointclouds_msg.header.frame_id = "map";
    filter_car_pointclouds_msg.header.stamp = ros::Time::now();
    pub_text_marker.publish(filter_car_pointclouds_msg);

    PreProcessCuda pre_process;
    pre_process.generateBevProjection();
    // // 2 设置深度图生成参数参数
    // // 2.1角度分辨率，水平最大采样角、垂直最大采样角
    // float angular_resolution = (float)(0.005f * (M_PI / 180.0f)); // 1度转弧度 ，1度的角分辨率
    // float max_angle_width = (float)(360.0f * (M_PI / 180.0f));    // 360.0度转弧度 /*模拟深度传感器在水平方向的最大采样角度:360°视角。*/
    // float max_angle_height = (float)(180.0f * (M_PI / 180.0f));   // 180.0度转弧度    /*模拟深度传感器在垂直方向的最大采样角度:180°视角。*/

    // // sensorPose定义模拟深度传感器的自由度位置： 横滚角roll、俯仰角pitch、偏航角yaw，默认初始值均为0。LASER_FRAME：X轴向前，Y轴向左，Z轴向上；
    // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 90.0f, 0.0f);
    // pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // CAMERA_FRAME：X轴向右，Y轴向下，Z轴向前；
    // float noise_level = 0.00;                                                          // noise_level=0.05，可以理解为，深度距离值是通过查询点半径为 5cm 的圆内包含的点用来平均计算而得到的。
    // float min_range = 0.1f;                                                            // min_range设置最小的获取距离，小于最小获取距离的位置为盲区
    // int border_size = 1;                                                               // border_size获得深度图像的边缘的宽度：在裁剪图像时，如果 border_size>O ，将在图像周围留下当前视点不可见点的边界

    // // 3 生成深度图
    // boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    // pcl::RangeImage &range_image = *range_image_ptr;
    // // 第一种方法
    // range_image.createFromPointCloud(pointCloud, angular_resolution, max_angle_width, max_angle_height, sensorPose, coordinate_frame, noise_level, min_range, border_size);
    // std::cout << range_image << "\n";
    // printf("get image success!");

    // // 显示深度图像
    // pcl::visualization::RangeImageVisualizer range_image_widget("Range image test rmy"); // 图像名称
    // // range_image_widget.setRangeImage(range_image);
    // range_image_widget.showRangeImage(range_image);                // 图像可视化方式显示深度图像
    // range_image_widget.setWindowTitle("Range image test rmy 223"); // 设置窗口名称
    // Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());

    // ros::Rate rate(10);
    // while (ros::ok())
    // {
    //     rate.sleep();
    // }
}
