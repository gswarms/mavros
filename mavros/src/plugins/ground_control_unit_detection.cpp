/*
 * Copyright 2014 Nuno Marques.
 * Copyright 2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief ground control unit detections
 * @file ground control unit detection.cpp
 * @author Iftach Naftaly <iftach@gswarms.com>
 *
 * @addtogroup plugin
 * @{
 */

#include "tf2_eigen/tf2_eigen.hpp"
#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros/setpoint_mixin.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief send ground control unit detections
 * @plugin ground_control_unit_detection
 *
 * Send ground control unit detections to the FCU.
 */
class GroundControlUnitDetections : public plugin::Plugin,
  private plugin::LocalPositionNEDCOVMixin<GroundControlUnitDetections>
{
public:
  explicit GroundControlUnitDetections(plugin::UASPtr uas_)
  : Plugin(uas_, "ground_control_unit")
  {
    node->declare_parameter("frame", std::string("local_origin_ned"));

    auto sensor_qos = rclcpp::SensorDataQoS();

    detection_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_control_unit/detection", sensor_qos, std::bind(
        &GroundControlUnitDetections::detection_cb, this,
        _1));
  }

  Subscriptions get_subscriptions() override
  {
    return { /* Rx disabled */};
  }

private:
  friend class plugin::LocalPositionNEDCOVMixin<GroundControlUnitDetections>;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr detection_sub;

  /* -*- mid-level helpers -*- */

  /**
   * @brief Send ground control unit detection to FCU.
   *
   * @warning Send only AFX AFY AFZ. ENU frame.
   */
  void send_detection(const rclcpp::Time & stamp, const Eigen::Vector3d & position_ned, const Eigen::Vector3d & velocity_ned, const uint8_t child_frame_id)
  {

    std::string _frame;
    node->get_parameter("frame", _frame);

    Eigen::Matrix<float, 1, 9> p_cov;
    p_cov.setZero();  // Initialize all elements to 0

    Eigen::Matrix<float, 1, 9> v_cov;
    v_cov.setZero();  // Initialize all elements to 0

    local_position_ned_cov(
      get_time_boot_ms(stamp),
      child_frame_id,
      position_ned,
      velocity_ned,
      p_cov,
      v_cov);
  }

  /* -*- callbacks -*- */

  void detection_cb(const nav_msgs::msg::Odometry::SharedPtr req)
  {
    Eigen::Vector3d position_ned;
    Eigen::Vector3d velocity_ned;
    uint8_t child_frame_id;

    child_frame_id = static_cast<uint8_t>(std::stoi(req->child_frame_id));

    position_ned[0] = req->pose.pose.position.x;
    position_ned[1] = req->pose.pose.position.y;
    position_ned[2] = req->pose.pose.position.z;

    velocity_ned[0] = req->twist.twist.linear.x;
    velocity_ned[1] = req->twist.twist.linear.y;
    velocity_ned[2] = req->twist.twist.linear.z;

    this->send_detection(req->header.stamp, position_ned, velocity_ned, child_frame_id);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::GroundControlUnitDetections)
