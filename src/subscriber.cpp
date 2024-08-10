#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <open3d/Open3D.h>
#include <tf/transform_datatypes.h>

#include "downsample.hpp"
#include "send_data.h"
#include "transform_data.hpp"


std::string createTimestampedFilename(std::string path) {
    //std::string path = "/home/kenji/pcd";
    // Get Now Time
    auto now = std::chrono::system_clock::now();
    auto inTimeT = std::chrono::system_clock::to_time_t(now);

    // Transfer Times to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&inTimeT), "%Y%m%d_%H%M%S");

    // Create FileName
    //std::string fileName = path + "/saved_cloud_pub_freq_" + std::to_string(2) + "_" + ss.str() + ".pcd";
    std::string file_name = path + "/pub_freq_" + std::to_string(2) + "_" + "voxel_size_01_nb_neighbors_30_std_ratio_01_" + ss.str() + ".pcd";
    return file_name;
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto source_downsampled = std::make_shared<open3d::geometry::PointCloud>();

    data_packet *data_packet_ptr = (data_packet*)malloc(sizeof(data_packet));
    data_packet_ptr->float_array_ptr = (double*)malloc(sizeof(double) * NUM_FLOATS * 3);

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Transform the point cloud data from PCL PointCloud to Open3D PointCloud
    for (const auto& point : pcl_cloud->points) {
        source->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // Downsample the point cloud and remove noise
    downsample(source, source_downsampled);

    data_packet_ptr->unix_time = time(NULL);
    data_packet_ptr->num_points = source_downsampled->points_.size();

    // Transform the point cloud data from Open3D PointCloud to data_packet
    transform_data(source_downsampled, data_packet_ptr);

    // Send the point cloud data to the server
    send_data("180.145.242.113", "1234", data_packet_ptr);

    sleep(1);

    // Free allocated memory
    free(data_packet_ptr->float_array_ptr);
    free(data_packet_ptr);

    // Timesstamped filename
    //std::string file_path = "/home/kenji/pcd";
    std::string file_path = "/home/kenji/pcd/downsampled_pcd";
    std::string file_name = createTimestampedFilename(file_path);

    // Save to a PCD file
    //pcl::io::savePCDFileASCII(fileName, *pcl_cloud);

    // Save to a PCD file
    /*
    bool result = open3d::io::WritePointCloud(file_name, *source_downsampled);
    if (result) {
        std::cout << "\e[32m" << "Successfully saved the point cloud." << "\e[m" << std::endl;
    } else {
        std::cerr << "\e[31m" << "Failed to save the point cloud." << "\e[m" << std::endl;
    }
    */

    //ROS_INFO("Saved %d data points to %s", cloud->size(), fileName.c_str());
}

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    /*
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    
    ROS_INFO("IMU Data - Orientation: Roll: [%f], Pitch: [%f], Yaw: [%f], Orientation: [%f, %f, %f, %f], Angular Velocity: [%f, %f, %f], Linear Acceleration: [%f, %f, %f]",
            roll, pitch, yaw,
            imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w,
            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    */
    double angular_velocity_x = imu_msg->angular_velocity.x * 180 / M_PI;
    double angular_velocity_y = imu_msg->angular_velocity.y * 180 / M_PI;
    double angular_velocity_z = imu_msg->angular_velocity.z * 180 / M_PI;

    ROS_INFO("IMU Data - Angular Velocity: [%f, %f, %f], Angular Velocity: [%f, %f, %f], Linear Acceleration: [%f, %f, %f]",
        angular_velocity_x, angular_velocity_y, angular_velocity_z,
        imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
        imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "livox_point_cloud_listener");
    ros::NodeHandle nh;

    // Create a ROS subscriber
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, Callback);

    //ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("livox/imu", 1, imu_callback);

    // Spin to continuously get data from callback
    ros::spin();

    return 0;
}