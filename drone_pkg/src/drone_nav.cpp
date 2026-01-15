#include "drone_nav.hpp"

using namespace std::chrono_literals;

DroneNav::DroneNav() : as2::Node("drone_nav")
{
  // Create clients for as2 action behaviors
  takeoff_client_ = rclcpp_action::create_client<TakeoffAction>(this, "/drone0/TakeoffBehavior");

  // speed handler for publishing velocities
  speed_handler_ = std::make_shared<SpeedHandler>(this, "drone0");

  // QoS y sync subscriptions
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  qos_profile.best_effort();

  pcl_subs_.subscribe(this, "/drone0/sensor_measurements/lidar_3d/points", qos_profile.get_rmw_qos_profile());
  pose_subs_.subscribe(this, "/drone0/self_localization/pose", qos_profile.get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(
        ApproxSyncPolicy(10), pcl_subs_, pose_subs_);
  sync_->registerCallback(
        std::bind(&DroneNav::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

  // publisher
  pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_dt", 10);

  timer_ = this->create_wall_timer(100ms, std::bind(&DroneNav::fly_callback, this));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool DroneNav::arm_drone()
{
  // --- --- CREATE CLIENTS FOR DRONE SERVICIES --- ---
  // arm service
  auto arm_client = this->create_client<std_srvs::srv::SetBool>("/drone0/set_arming_state");
  if (!arm_client->wait_for_service(3s)) {
    RCLCPP_ERROR(this->get_logger(), "service /drone0/set_arming_state not available");
    this->mission_started = false;
    return false;
  }

  auto arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // set arm to true
  arm_request->data = true;
  auto arming = arm_client->async_send_request(arm_request);

  // off board service
  auto offboard_client = this->create_client<std_srvs::srv::SetBool>("/drone0/set_offboard_mode");
  if (!offboard_client->wait_for_service(3s)) {
    RCLCPP_ERROR(this->get_logger(), "service /drone0/set_offboard_mode not available");
    this->mission_started = false;

    return false;
  }

  auto ob_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  // set off board to true
  ob_request->data = true;
  auto offboarding = offboard_client->async_send_request(ob_request);
  
  RCLCPP_INFO(this->get_logger(), "arming and offboard commands sent");
  return true;
}

void DroneNav::run_mission()
{
  RCLCPP_INFO(this->get_logger(), "arming drone...");
  if (!arm_drone()) {
    RCLCPP_ERROR(this->get_logger(), "failed to send arm/offboard commands");
    return;
  }
  
  // wait for arming to complete before takeoff
  rclcpp::sleep_for(2s);

  // envío de goal de takeoff
  if (!takeoff_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "takeoff action not available");
    return;
  }

  auto goal_msg = TakeoffAction::Goal();
  goal_msg.takeoff_height = 0.75;

  auto send_goal_options = rclcpp_action::Client<TakeoffAction>::SendGoalOptions();
  send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<TakeoffAction>::WrappedResult & result){
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) 
    {
      RCLCPP_INFO(this->get_logger(), "take off SUCCESS");
      this->act_state = FLYING;
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "take off ERROR!");
      this->act_state = TAKE_OFF;
      this->mission_started = false;
    }
  };

  takeoff_client_->async_send_goal(goal_msg, send_goal_options);
}

void DroneNav::fly_callback()
{
  // --- --- FLY DRONE --- ---
  if (this->act_state != FLYING) {
    RCLCPP_INFO(this->get_logger(), "NOT FLYING, BUSY");
    return;
  }

  std::string frame_id = "drone0/odom"; 
  float speed_module = 0.75;

  if (fly_forward) {
    RCLCPP_INFO(this->get_logger(), "FLYING FORWARD");

    // use actual orientation to compute linear velocity (trigonometry)
    float vx = speed_module * std::cos(this->current_yaw_);
    float vy = speed_module * std::sin(this->current_yaw_);
    
    // send velocities to as2 motion controller
    speed_handler_->sendSpeedCommandWithYawSpeed(frame_id, vx, vy, 0.0, 0.0);

  } else {
    RCLCPP_INFO(this->get_logger(), "TURNING");
    if (turn_left) {
      //  turn left
      speed_handler_->sendSpeedCommandWithYawSpeed(frame_id, 0.0, 0.0, 0.0, speed_module);
    } else {
      //  turn right
      speed_handler_->sendSpeedCommandWithYawSpeed(frame_id, 0.0, 0.0, 0.0, -speed_module);
    }
  }
}
void DroneNav::synced_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
{
  // --- --- INITIALIZE DRONE --- ---
  if (this->act_state == TAKE_OFF) {
    RCLCPP_INFO(this->get_logger(), "waiting for takeoff to complete...");
    if (!this->mission_started) {
        this->mission_started = true;
        run_mission();
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ> in_cloud;
    pcl::fromROSMsg(*cloud_msg, in_cloud);

    if (in_cloud.empty()) {
      RCLCPP_INFO(this->get_logger(), "NOT RECEIVING CLOUD");
      return;
    }
    
    // --- --- FILTER INPUT CLOUD --- --- 
    pcl::PointCloud<pcl::PointXYZ> out_cloud;
    const float DRONE_RADIUS = 0.25f; // ignore drone points for PCL creation
    const float Z_MIN = 0.2f; // ignore low points for navigation
    float total_dist_F = 0.0f, total_dist_L = 0.0f, total_dist_R = 0.0f;
    int F_points = 0, L_points = 0, R_points = 0;

    for (const auto & point: in_cloud) {
      if (isinf(point.z)) continue;
      // skip drone points -> inside drone radius
      float dist_point = std::sqrt(point.x*point.x + point.y*point.y);
      if (dist_point < DRONE_RADIUS) continue;

      // add every point except drone-related or infinite
      out_cloud.push_back(point);
      // discard heigth<drone_heigth points for nav
      if (point.z < Z_MIN)  continue;

      // use inverse value for 'Harmonic Mean'
      float inv_dist_point = 1.0/dist_point;
      // get angle between drone and measured point
      float theta = std::atan2(point.y, point.x);
      // if theta aprox 0rad -> FRONT
      if (std::fabs(theta) < 0.20f) {
        total_dist_F += inv_dist_point;
        F_points++;
      }
      // if theta aprox pi/2 rad -> SIDES
      if ((theta > (M_PI/2 - 0.20f)) && (theta < (M_PI/2 + 0.20f))){
        // LEFT
        total_dist_L += inv_dist_point;
        L_points++;
      }
      if ((theta > (-M_PI/2 - 0.20f)) && (theta < (-M_PI/2 + 0.20f))){
        // RIGHT
        total_dist_R += inv_dist_point;
        R_points++;
      }
    }

    // --- --- DECISION CONTROL --- --- 
    // check proximity with points
    // float mu_dist = F_points / total_dist_F;
    // float mu_dist_L = L_points / total_dist_L;
    // float mu_dist_R = R_points / total_dist_R;
    float mu_dist   = (total_dist_F > 0.001f) ? (F_points / total_dist_F) : 0.5f;
    float mu_dist_L = (total_dist_L > 0.001f) ? (L_points / total_dist_L) : 0.5f;
    float mu_dist_R = (total_dist_R > 0.001f) ? (R_points / total_dist_R) : 0.5f;
    
    const float SIDE_SAFETY_DIST = 0.65f; 
    const float FRONT_STOP_DIST = 2.0f;
    
    // OBSTACLE LEFT -> turn right
    if (mu_dist_L < SIDE_SAFETY_DIST) {
      RCLCPP_WARN(this->get_logger(), "MURO IZQUIERDA %f", mu_dist_L);
      this->fly_forward = false;
      this->turn_left = false; // false = girar derecha
    }
    // OBSTACLE RIGHT -> turn left
    else if (mu_dist_R < SIDE_SAFETY_DIST) {
      RCLCPP_WARN(this->get_logger(), "MURO DERECHA %f", mu_dist_R);
      this->fly_forward = false;
      this->turn_left = true; // true = girar izquierda
    // OBSTACLE FRONT -> turn to direction with furthest obstacle
    } else if (mu_dist < FRONT_STOP_DIST) {
      RCLCPP_INFO(this->get_logger(), "FRENTE", mu_dist);
      this->fly_forward = false;
      // block turn direction -> avoid local min problem
      if (!this->avoiding_frontal_) {
        this->avoiding_frontal_ = true;
        // if diff is min, force left turn
        if (std::abs(mu_dist_L - mu_dist_R) < 0.2f) {
          this->locked_turn_left_ = true; 
        } else {
          // furthest obstacle
          this->locked_turn_left_ = (mu_dist_L > mu_dist_R);
        }
        RCLCPP_INFO(this->get_logger(), "bloqueando giro");
      }
      this->turn_left = this->locked_turn_left_;
    }
    // NO FRONT OBSTACLE
    else {
      // stop turning if there is no longer an obstacle
      if (mu_dist > FRONT_STOP_DIST + 0.2f) {
        this->fly_forward = true;
        if (this->avoiding_frontal_) {
          this->avoiding_frontal_ = false;
          RCLCPP_INFO(this->get_logger(), "reset");
        }
      } else {
        // there is no obstacle, but its still close
        this->fly_forward = false;
        this->turn_left = this->locked_turn_left_;
      }
    }
      
    // --- --- PROCESS DRONE'S ORIENTATION --- ---
    double qx = pose_msg->pose.orientation.x;
    double qy = pose_msg->pose.orientation.y;
    double qz = pose_msg->pose.orientation.z;
    double qw = pose_msg->pose.orientation.w;

    // get EULER from QUATERNION
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    // change yaw to actual
    this->current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    // --- --- publish cloud --- ---
    sensor_msgs::msg::PointCloud2 local_cloud_msg;
  //   sensor_msgs::msg::PointCloud2 global_cloud_msg;
    pcl::toROSMsg(out_cloud, local_cloud_msg);
  //   local_cloud_msg.header = cloud_msg->header;

  //   std::string target_frame = "map"; 

//     geometry_msgs::msg::TransformStamped transform;
//     transform = tf_buffer_->lookupTransform(
//       "drone0/map",
//       cloud_msg->header.frame_id,
//       tf2::TimePointZero);
//     tf2::doTransform(local_cloud_msg, global_cloud_msg, transform);

    local_cloud_msg.header = cloud_msg->header;
    // local_cloud_msg.header.stamp = local_cloud_msg.header.stamp;

    pcl_pub_->publish(local_cloud_msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneNav>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}