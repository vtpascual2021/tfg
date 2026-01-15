#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class CreateMap : public rclcpp::Node
{
public:
  CreateMap();
private:
  // --- methods ---
  
  // callbacks
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in);

  // --- attributes ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // map memory (acc. cloud)
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  // voxel filter -> downsampling
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

  rclcpp::TimerBase::SharedPtr timer_;
  // publish in '/map' 
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  // subscribe to '/map_dt' topic
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
};
