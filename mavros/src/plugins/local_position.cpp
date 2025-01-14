/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Local position plugin.
 * @plugin local_position
 *
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class LocalPositionPlugin : public plugin::Plugin
{
public:
  explicit LocalPositionPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "local_position"),
    tf_send(false),
    has_local_position_ned(false),
    has_local_position_ned_cov(false)
  {
    enable_node_watch_parameters();

    // header frame_id.
    // default to map (world-fixed, ENU as per REP-105).
    node_declare_and_watch_parameter(
      "frame_id", "map", [&](const rclcpp::Parameter & p) {
        frame_id = p.as_string();
      });

    // Important tf subsection
    // Report the transform from world to base_link here.
    node_declare_and_watch_parameter(
      "tf.send", false, [&](const rclcpp::Parameter & p) {
        tf_send = p.as_bool();
      });
    node_declare_and_watch_parameter(
      "tf.frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });
    node_declare_and_watch_parameter(
      "tf.child_frame_id", "base_link", [&](const rclcpp::Parameter & p) {
        tf_child_frame_id = p.as_string();
      });

    auto sensor_qos = rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::BestEffort);
    local_odom = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&LocalPositionPlugin::handle_local_position_ned),
      make_handler(&LocalPositionPlugin::handle_local_position_ned_cov)
    };
  }

private:

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom;

  std::string frame_id;                 //!< frame for Pose
  std::string tf_frame_id;              //!< origin for TF
  std::string tf_child_frame_id;        //!< frame for TF
  std::atomic<bool> tf_send;
  std::atomic<bool> has_local_position_ned;
  std::atomic<bool> has_local_position_ned_cov;

  void publish_tf(nav_msgs::msg::Odometry & odom)
  {
    if (tf_send) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = odom.header.stamp;
      transform.header.frame_id = tf_frame_id;
      transform.child_frame_id = tf_child_frame_id;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      transform.transform.rotation = odom.pose.pose.orientation;
      uas->tf2_broadcaster.sendTransform(transform);
    }
  }

  void handle_local_position_ned(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::LOCAL_POSITION_NED & pos_ned,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    has_local_position_ned = true;

    auto odom = nav_msgs::msg::Odometry();
    odom.header = uas->synchronized_header(frame_id, pos_ned.time_boot_ms);
    odom.child_frame_id = tf_child_frame_id;

    odom.pose.pose.position.x = pos_ned.x;
    odom.pose.pose.position.y = pos_ned.y;
    odom.pose.pose.position.z = pos_ned.z;

    
    odom.twist.twist.linear.x = pos_ned.vx;
    odom.twist.twist.linear.y = pos_ned.vy;
    odom.twist.twist.linear.z = pos_ned.vz;

    local_odom->publish(odom);
   }

  void handle_local_position_ned_cov(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::LOCAL_POSITION_NED_COV & pos_ned,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    has_local_position_ned_cov = true;


    auto odom = nav_msgs::msg::Odometry();
    odom.header = uas->synchronized_header(frame_id, pos_ned.time_usec);
    odom.child_frame_id = tf_child_frame_id;

    odom.pose.pose.position.x = pos_ned.x;
    odom.pose.pose.position.y = pos_ned.y;
    odom.pose.pose.position.z = pos_ned.z;

    
    odom.twist.twist.linear.x = pos_ned.vx;
    odom.twist.twist.linear.y = pos_ned.vy;
    odom.twist.twist.linear.z = pos_ned.vz;


    // publish odom always
    local_odom->publish(odom);

  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::LocalPositionPlugin)
