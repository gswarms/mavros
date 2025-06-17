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

    auto sensor_qos = rclcpp::SensorDataQoS();

    miss_distance_pub = node->create_publisher<std_msgs::msg::Float32>("interception/miss_distance", 1);
    tgo_pub = node->create_publisher<std_msgs::msg::Float32>("interception/tgo", 1);
    position_std_norm_pub = node->create_publisher<std_msgs::msg::Float32>("interception/position_std_norm", 1);
    substate_pub = node->create_publisher<std_msgs::msg::Int32>("interception/substate", 1);
    target_detected_pub = node->create_publisher<std_msgs::msg::Bool>("interception/target_detected", 1);
    estimated_relative_position_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "interception/estimated_relative_position", 1);
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
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr              substate_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               target_detected_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  estimated_relative_position_pub;


  void handle_interception(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::INTERCEPTION_DATA & interception, plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    auto miss_distance_msg = std_msgs::msg::Float32();
    miss_distance_msg.data = interception.miss_distance;
    miss_distance_pub->publish(miss_distance_msg);

    auto tgo_msg = std_msgs::msg::Float32();
    tgo_msg.data = interception.tgo;
    tgo_pub->publish(tgo_msg);

    auto position_std_norm_msg = std_msgs::msg::Float32();
    position_std_norm_msg.data = interception.position_std_norm;
    position_std_norm_pub->publish(position_std_norm_msg);

    auto substate_msg = std_msgs::msg::Int32();
    substate_msg.data = interception.substate;
    substate_pub->publish(substate_msg);

    auto target_detected_msg = std_msgs::msg::Bool();
    target_detected_msg.data = interception.target_detected;
    target_detected_pub->publish(target_detected_msg);

    auto estimated_relative_position_msg = std_msgs::msg::Float32MultiArray();
    estimated_relative_position_msg.data.resize(3);
    estimated_relative_position_msg.data[0] = interception.estimated_relative_position[0];
    estimated_relative_position_msg.data[1] = interception.estimated_relative_position[1];
    estimated_relative_position_msg.data[2] = interception.estimated_relative_position[2];
    estimated_relative_position_pub->publish(estimated_relative_position_msg);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::InterceptionPlugin)
