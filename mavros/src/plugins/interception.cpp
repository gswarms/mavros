/*
 * Copyright 2015 Iftach Naftaly <iftach@lulav.space>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief Interception plugin
 * @file interception.cpp
 * @author Iftach Naftaly <iftach@lulav.space>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace mavros
{
namespace std_plugins
{

/**
 * @brief Interception plugin.
 * @plugin Interception
 */
class InterceptionPlugin : public plugin::Plugin
{
public:
  explicit InterceptionPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "interception")
  {
    enable_node_watch_parameters();

    miss_distance_pub               = node->create_publisher<std_msgs::msg::Float32>("interception/miss_distance", 1);
    tgo_pub                         = node->create_publisher<std_msgs::msg::Float32>("interception/tgo", 1);
    position_std_norm_pub           = node->create_publisher<std_msgs::msg::Float32>("interception/position_std_norm", 1);
    target_time_delay_pub           = node->create_publisher<std_msgs::msg::Float32>("interception/target_time_delay", 1);
    substate_pub                    = node->create_publisher<std_msgs::msg::Int32>("interception/substate", 1);
    target_detected_pub             = node->create_publisher<std_msgs::msg::Bool>("interception/target_detected", 1);
    interception_active_pub         = node->create_publisher<std_msgs::msg::Bool>("interception/interception_active", 1);
    target_estimator_active_pub     = node->create_publisher<std_msgs::msg::Bool>("interception/target_estimator_active", 1);
    camera_driver_active_pub        = node->create_publisher<std_msgs::msg::Bool>("interception/camera_driver_active", 1);
    monitor_active_pub              = node->create_publisher<std_msgs::msg::Bool>("interception/monitor_active", 1);
    recorder_active_pub             = node->create_publisher<std_msgs::msg::Bool>("interception/recorder_active", 1);
    detector_active_pub             = node->create_publisher<std_msgs::msg::Bool>("interception/detector_active", 1);
    osd_active_pub                  = node->create_publisher<std_msgs::msg::Bool>("interception/osd_active", 1);
    estimated_relative_position_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "interception/estimated_relative_position", 1);

    RCLCPP_INFO(
      node->get_logger(),
      "Interception plugin initialized!");
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&InterceptionPlugin::handle_interception),
    };
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            miss_distance_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            tgo_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            position_std_norm_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr            target_time_delay_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr              substate_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               target_detected_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               interception_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               target_estimator_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               camera_driver_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               monitor_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               recorder_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               detector_active_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               osd_active_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  estimated_relative_position_pub;


  void handle_interception(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::INTERCEPTION_DATA & interception, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {

    auto miss_distance_msg                        = std_msgs::msg::Float32();
    auto tgo_msg                                  = std_msgs::msg::Float32();
    auto position_std_norm_msg                    = std_msgs::msg::Float32();
    auto target_time_delay_msg                   = std_msgs::msg::Float32();
    auto substate_msg                             = std_msgs::msg::Int32();
    auto target_detected_msg                      = std_msgs::msg::Bool();
    auto interception_active_msg                  = std_msgs::msg::Bool();
    auto target_estimator_active_msg              = std_msgs::msg::Bool();
    auto camera_driver_active_msg                 = std_msgs::msg::Bool();
    auto monitor_active_msg                       = std_msgs::msg::Bool();
    auto recorder_active_msg                      = std_msgs::msg::Bool();
    auto detector_active_msg                      = std_msgs::msg::Bool();
    auto osd_active_msg                           = std_msgs::msg::Bool();
    auto estimated_relative_position_msg          = std_msgs::msg::Float32MultiArray();

    miss_distance_msg.data                        = interception.miss_distance;
    tgo_msg.data                                  = interception.tgo;
    position_std_norm_msg.data                    = interception.position_std_norm;
    target_time_delay_msg.data                   = interception.target_time_delay;
    substate_msg.data                             = interception.substate;
    target_detected_msg.data                      = interception.target_detected;
    interception_active_msg.data                  = interception.interceptor_active;
    target_estimator_active_msg.data              = interception.target_estimation_active;
    camera_driver_active_msg.data                 = interception.camera_driver_active;
    monitor_active_msg.data                       = interception.monitor_active;
    recorder_active_msg.data                      = interception.recorder_active;
    detector_active_msg.data                      = interception.detector_active;
    osd_active_msg.data                           = interception.osd_active;

    estimated_relative_position_msg.data.resize(3);
    estimated_relative_position_msg.data[0]       = interception.estimated_relative_position[0];
    estimated_relative_position_msg.data[1]       = interception.estimated_relative_position[1];
    estimated_relative_position_msg.data[2]       = interception.estimated_relative_position[2];

    miss_distance_pub->publish(miss_distance_msg);
    tgo_pub->publish(tgo_msg);
    position_std_norm_pub->publish(position_std_norm_msg);
    target_time_delay_pub->publish(target_time_delay_msg);
    substate_pub->publish(substate_msg);
    target_detected_pub->publish(target_detected_msg);
    interception_active_pub->publish(interception_active_msg);
    target_estimator_active_pub->publish(target_estimator_active_msg);
    camera_driver_active_pub->publish(camera_driver_active_msg);
    monitor_active_pub->publish(monitor_active_msg);
    recorder_active_pub->publish(recorder_active_msg);
    detector_active_pub->publish(detector_active_msg);
    osd_active_pub->publish(osd_active_msg);
    
    estimated_relative_position_pub->publish(estimated_relative_position_msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::InterceptionPlugin)
