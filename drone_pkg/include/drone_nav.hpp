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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "as2_core/node.hpp"
#include "as2_msgs/action/takeoff.hpp"
// #include "as2_msgs/msg>
#include "as2_msgs/action/go_to_waypoint.hpp"
#include "as2_motion_reference_handlers/speed_motion.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      geometry_msgs::msg::PoseStamped
    > ApproxSyncPolicy;

class DroneNav : public as2::Node
{
public:
  DroneNav();

  using TakeoffAction = as2_msgs::action::Takeoff;
  using TakeoffClient = rclcpp_action::Client<TakeoffAction>;

  using SpeedHandler = as2::motionReferenceHandlers::SpeedMotion;

private:
  // --- methods ---
  bool arm_drone();
  void run_mission();

  // callbacks
  void fly_callback();
  void synced_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_subs,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_subs);

  // --- attributes ---
  enum DRONE_STATE {
    TAKE_OFF = 0,
    FLYING,
    LANDING
  };

  DRONE_STATE act_state = TAKE_OFF;

  int counter_turn = 0;
  double current_yaw_ = 0.0;

  bool mission_started = false;      // true = drone ready to fly
  bool fly_forward = true;           // true = flying toward actual +x
  bool turn_left = false;            // true = turn left when turning

  bool avoiding_frontal_ = false;    // true = navigation avoiding frontal obstacle
  bool locked_turn_left_ = false;    // true = turn left if avoiding_frontal_ is true
  
  rclcpp_action::Client<TakeoffAction>::SharedPtr takeoff_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  // publish in '/map_dt' 
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  // subscribe to '/drone0/sensor_measurements/lidar_3d/points' topic
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_subs_;
  // subscribe to '/drone0/self_localization/pose'
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_subs_;

  std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<SpeedHandler> speed_handler_;
};
