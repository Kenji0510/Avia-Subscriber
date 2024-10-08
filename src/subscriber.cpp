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
#include <vector>
#include <eigen3/Eigen/Dense>

#include "downsample.hpp"
#include "send_data.h"
#include "transform_data.hpp"


data_packet *data_packet_ptr = NULL;
const int MAX_DATA_SIZE = 100;
Eigen::Vector3d integrated_angle(0.0, 0.0, 0.0);
const double time_interval = 1.0 / 200.0;
size_t count = 0;

std::vector<Eigen::Vector3d> angular_velocity_array;
Eigen::Vector3d max_angular_velocity(0.0, 0.0, 0.0);


void save_angular_velocity_data() {
    std::ofstream file;
    file.open("/home/kenji/ws_livox/src/lidar_subscriber/src/angle_velocity_data.txt");
    for (int i = 0; i < angular_velocity_array.size(); i++) {
        file << angular_velocity_array[i](0) << ", " << angular_velocity_array[i](1) << ", " << angular_velocity_array[i](2) << std::endl;
    }
    file.close();
}

void check_max_value(Eigen::Vector3d angular_velocity) {
    if (angular_velocity(0) > max_angular_velocity(0)) {
        max_angular_velocity(0) = angular_velocity(0);
    }
    if (angular_velocity(1) > max_angular_velocity(1)) {
        max_angular_velocity(1) = angular_velocity(1);
    }
    if (angular_velocity(2) > max_angular_velocity(2)) {
        max_angular_velocity(2) = angular_velocity(2);
    }
}

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

    if (data_packet_ptr == NULL) {
        data_packet_ptr = (data_packet*)malloc(sizeof(data_packet));
        data_packet_ptr->float_array_ptr = (double*)malloc(sizeof(double) * NUM_FLOATS * 3);
    }
    
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
    data_packet_ptr->angle_velocity_x = integrated_angle(0);
    data_packet_ptr->angle_velocity_y = integrated_angle(1);
    data_packet_ptr->angle_velocity_z = integrated_angle(2);

    // Transform the point cloud data from Open3D PointCloud to data_packet
    transform_data(source_downsampled, data_packet_ptr);

    // Send the point cloud data to the server
    int result = send_data("180.145.242.113", "1234", data_packet_ptr);

    // Free allocated memory
    //free(data_packet_ptr->float_array_ptr);
    //free(data_packet_ptr);

    // Timesstamped filename
    //std::string file_path = "/home/kenji/pcd";
    std::string file_path = "/home/kenji/pcd/downsampled_pcd";
    std::string file_name = createTimestampedFilename(file_path);

    count += 1;

    ROS_INFO("IMU Angular Velocity Data (Degree):");
    /*
    for (int i = 0; i < current_index; i++) {
        ROS_INFO("Index: [%d], Angular Velocity: [%f, %f, %f]", i, angular_velocity_x_array[i], angular_velocity_y_array[i], angular_velocity_z_array[i]);
    }
    current_index = 0;
    */
    //ROS_INFO("Integrated Angle: [%f, %f, %f]", integrated_angle(0), integrated_angle(1), integrated_angle(2));
    ROS_INFO("Count: %ld Integrated Angle: [%f, %f, %f]", count, integrated_angle(0), integrated_angle(1), integrated_angle(2));

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
    /*
    double angular_velocity_x = imu_msg->angular_velocity.x * 180 / M_PI;
    double angular_velocity_y = imu_msg->angular_velocity.y * 180 / M_PI;
    double angular_velocity_z = imu_msg->angular_velocity.z * 180 / M_PI;

    if (current_index < MAX_DATA_SIZE) {
        angular_velocity_x_array[current_index] = angular_velocity_x;
        angular_velocity_y_array[current_index] = angular_velocity_y;
        angular_velocity_z_array[current_index] = angular_velocity_z;
        current_index++;
    } 
    */

    Eigen::Vector3d angular_velocity(
            imu_msg->angular_velocity.x * 180 / M_PI,
            imu_msg->angular_velocity.y * 180 / M_PI,
            imu_msg->angular_velocity.z * 180 / M_PI
    );

    //check_max_value(angular_velocity);
    //std::cout << "Max Angular Velocity: " << max_angular_velocity << std::endl;
    //angular_velocity_array.push_back(angular_velocity);
    // Filter out measurement errors
    if (angular_velocity(0) < 1.65) {
        angular_velocity(0) = 0.0;
    }
    if (angular_velocity(1) < 2.5) {
        angular_velocity(1) = 0.0;
    }
    if (angular_velocity(2) < 1.6) {
        angular_velocity(2) = 0.0;
    }

    std::cout << "Angular Velocity: " << angular_velocity << std::endl;

    integrated_angle += angular_velocity * time_interval;

    /*
    ROS_INFO("IMU Data - Angular Velocity(Degree): [%f, %f, %f], Angular Velocity: [%f, %f, %f], Linear Acceleration: [%f, %f, %f]",
        angular_velocity_x, angular_velocity_y, angular_velocity_z,
        imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z,
        imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    */
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "livox_point_cloud_listener");
    ros::NodeHandle nh;

    // Create a ROS subscriber
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, Callback);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("livox/imu", 1, imu_callback);

    // Spin to continuously get data from callback
    ros::spin();

    if (data_packet_ptr != NULL) {
        free(data_packet_ptr->float_array_ptr);
        free(data_packet_ptr);
        std::cout << "\e[33m" << "Freed allocated memory." << "\e[m" << std::endl;
        //save_angular_velocity_data();
    }

    return 0;
}