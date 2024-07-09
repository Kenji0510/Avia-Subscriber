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
#include <tf/transform_datatypes.h>

std::string createTimestampedFilename() {
    std::string path = "/home/kenji/pcd";
    // Get Now Time
    auto now = std::chrono::system_clock::now();
    auto inTimeT = std::chrono::system_clock::to_time_t(now);

    // Transfer Times to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&inTimeT), "%Y%m%d_%H%M%S");

    // Create FileName
    std::string fileName = path + "/saved_cloud_pub_freq_" + std::to_string(2) + "_" + ss.str() + ".pcd";
    return fileName;
}

void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Timesstamped filename
    std::string fileName = createTimestampedFilename();

    // Save to a PCD file
    pcl::io::savePCDFileASCII(fileName, *cloud);

    ROS_INFO("Saved %d data points to %s", cloud->size(), fileName.c_str());
}

void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
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

    return 0;
}