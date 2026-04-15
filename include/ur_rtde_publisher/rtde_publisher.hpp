// Copyright 2026, Universal Robots A/S

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.

//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.

//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ur_client_library/rtde/rtde_client.h"
#include "ur_rtde_publisher/rtde_converter.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <example_interfaces/msg/u_int32.hpp>
#include <example_interfaces/msg/int32.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/u_int64.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <example_interfaces/msg/byte_multi_array.hpp>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_status.hpp>
#include <example_interfaces/msg/int32_multi_array.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <map>
#include <memory>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace ur_rtde_publisher
{
/**
 * @brief Static configuration for one RTDE variable as defined by the YAML mapping.
 */
struct VariableConfig
{
  std::string name;
  std::string rtde_type;
  std::string output_type;
  std::string topic;
  std::string frame_id;
};

/**
 * @brief Runtime representation of an active RTDE variable.
 */
struct ActiveVariable
{
  std::string name;
  VariableConfig config;
  std::function<void(const urcl::rtde_interface::DataPackage&, const rclcpp::Time&)> publish;
};

/**
 * @brief Loads the YAML mapping, creates ROS publishers on demand for a given recipe,
 *        and publishes fields extracted from UR RTDE data packages.
 */
class RTDEPublisher
{
public:
  /**
   * @brief Constructor.
   * @param node Reference to the ROS2 node.
   * @param config_path Path to the YAML mapping file ('rtde_map.yaml').
   * @param tf_prefix Prefix to add to the frame_id.
   */
  RTDEPublisher(rclcpp::Node& node, const std::string& config_path, const std::string& tf_prefix);

  /**
   * @brief Create publishers for the variables specified by the recipe.
   * @param recipe List of RTDE variable names to activate/publish.
   */
  void createPublishersForRecipe(const std::vector<std::string>& recipe);

  /**
   * @brief Publish all active fields extracted from the given RTDE package.
   * @param pkg UR RTDE data package (UR Client Library).
   * @param packet_time Time when RTDE returned data.
   */
  void publish(const urcl::rtde_interface::DataPackage& pkg, rclcpp::Time& packet_time);

  /**
   * @brief Get the list of effective keys activated after applying the recipe.
   * @return A copy of active variable names (keys of 'active_variables_').
   */
  const std::vector<std::string> effective_keys() const;

private:
  /// @brief Reference to the ROS2 node used for creating publishers and logging.
  rclcpp::Node& node_;

  /// @brief Index of all variables defined in YAML (name -> config).
  std::vector<VariableConfig> all_variables_;

  /// @brief Active subset after applying the recipe (name -> config).
  std::vector<ActiveVariable> active_variables_;

  /// @brief Unordered Set to ensure no duplicated variables names are used.
  std::unordered_set<std::string> seen_vars;

  /**
   * @brief Load and validate the mapping YAML, filling 'all_variables_'.
   * @param config_path Path where the yaml file is located.
   * @param tf_prefix Prefix to add to each frame_id.
   */
  void loadConfig(const std::string& config_path, const std::string& tf_prefix);

  /**
   * @brief Create ROS2 publishers for all 'active_variables_'.
   * @return true if it successfully creates the publisher, false otherwise.
   */
  bool createPublisher(ActiveVariable& av);

  /**
   * @brief Publish a single variable by reading from the RTDE package and converting to its ROS type.
   * @param var_name RTDE variable name.
   * @param pkg UR RTDE package to extract data from.
   */
  void publishVariable(const ActiveVariable& av, const urcl::rtde_interface::DataPackage& pkg,
                       rclcpp::Time& packet_time);

  /**
   * @brief Expand a variable pattern with range (e.g., "var_<0-5>") into individual variables.
   * @param pattern The pattern string.
   * @return Vector of expanded variable names.
   */
  std::vector<std::string> expandRange(const std::string& pattern);
};

}  // namespace ur_rtde_publisher