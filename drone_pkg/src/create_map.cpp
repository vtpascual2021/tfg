#include "create_map.hpp"

CreateMap::CreateMap() : rclcpp::Node ("map_create_node")
{
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  // subscribe to the pcl
  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/map_dt", qos, std::bind(&CreateMap::callback, this, std::placeholders::_1));
  // publish map
  pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  voxel_filter_.setLeafSize(0.1f, 0.1f, 0.1f);
}

void CreateMap::callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in)
{
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  std::string target_frame = "map";
  
  // input frame is map
  if (cloud_in->header.frame_id == target_frame) {
      transformed_cloud = *cloud_in;
  } 
  else {
      // get transform
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
        "drone0/map", // to
        cloud_in->header.frame_id, // from
        tf2::TimePointZero
      );
      // transform frame from cloud
      tf2::doTransform(*cloud_in, transformed_cloud, transform);
  }

  pcl::PointCloud<pcl::PointXYZ> current_pcl;
  pcl::fromROSMsg(transformed_cloud, current_pcl);

  if (current_pcl.empty()) return;
  // increment map memory
  *global_map_ += current_pcl;

  // downsample with VoxelFilter
  voxel_filter_.setInputCloud(global_map_);
  voxel_filter_.filter(*global_map_);

  // publish
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*global_map_, output_msg);
  
  // 'map' = 'drone0/map'
  output_msg.header.frame_id = target_frame;
  output_msg.header.stamp = cloud_in->header.stamp;
  pcl_pub_->publish(output_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CreateMap>());
  rclcpp::shutdown();
  return 0;
}