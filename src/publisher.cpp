// Copyright 2026, Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
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

#include "ur_rtde_publisher/publisher.hpp"

namespace ur_rtde_publisher
{
Publisher::Publisher(rclcpp::Node& node, const std::string& config_path) : node_(node)
{
  loadConfig(config_path);
}

void Publisher::loadConfig(const std::string& config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);
  all_variables_.clear();

  const auto& rtde_vars = config["rtde_variables"];

  for (auto group_it = rtde_vars.begin(); group_it != rtde_vars.end(); ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    const auto& group_config = group_it->second;

    std::string rtde_type = group_config["rtde_type"].as<std::string>();
    std::string output_type = group_config["output_type"].as<std::string>();
    const auto& variables = group_config["variables"];

    for (const auto& var_node : variables) {
      std::string var_pattern = var_node.as<std::string>();
      auto expanded_vars = expandRange(var_pattern);

      for (const auto& var_name : expanded_vars) {
        VariableConfig vc;
        vc.name = var_name;
        vc.rtde_type = rtde_type;
        vc.output_type = output_type;
        vc.topic = "rtde/" + var_name;

        all_variables_.push_back(vc);
      }
    }
  }

  RCLCPP_INFO(node_.get_logger(), "YAML file loaded successfully.");
}

std::vector<std::string> Publisher::expandRange(const std::string& pattern)
{
  size_t pos = pattern.find('<');
  if (pos == std::string::npos) {
    return { pattern };
  }

  std::string prefix = pattern.substr(0, pos);
  size_t end_pos = pattern.find('>', pos);
  if (end_pos == std::string::npos) {
    return { pattern };
  }

  std::string range_str = pattern.substr(pos + 1, end_pos - pos - 1);
  size_t dash_pos = range_str.find('-');
  if (dash_pos == std::string::npos) {
    return { pattern };
  }

  try {
    int start = std::stoi(range_str.substr(0, dash_pos));
    int end = std::stoi(range_str.substr(dash_pos + 1));

    std::vector<std::string> expanded;
    for (int i = start; i <= end; ++i) {
      expanded.push_back(prefix + std::to_string(i));
    }
    return expanded;
  } catch (const std::exception&) {
    return { pattern };
  }
}

void Publisher::createPublishersForRecipe(const std::vector<std::string>& recipe)
{
  active_variables_.clear();

  RCLCPP_INFO(node_.get_logger(), "Creating publishers for recipe with %zu variables", recipe.size());

  for (const auto& var_name : recipe) {
    auto it = std::find_if(all_variables_.begin(), all_variables_.end(),
                           [&](const VariableConfig& vc) { return vc.name == var_name; });
    if (it == all_variables_.end()) {
      RCLCPP_WARN(node_.get_logger(), "Variable %s from recipe not found in config, skipping", var_name.c_str());
      continue;
    }
    ActiveVariable av;
    av.name = var_name;
    av.config = *it;

    createPublisher(av);

    active_variables_.push_back(av);
  }
}

void Publisher::createPublisher(ActiveVariable& av)
{
  auto qos = rclcpp::SensorDataQoS();

  if (av.config.output_type == "geometry_msgs/TwistStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::TwistStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created TwistStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/WrenchStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::WrenchStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created WrenchStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/PoseStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::PoseStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created PoseStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/AccelStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::AccelStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created AccelStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/PointStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::PointStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created PointStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/Vector3Stamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Vector3Stamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "geometry_msgs/InertiaStamped") {
    av.publisher = node_.create_publisher<geometry_msgs::msg::InertiaStamped>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created InertiaStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/UInt32") {
    av.publisher = node_.create_publisher<example_interfaces::msg::UInt32>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created UInt32 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/Float64") {
    av.publisher = node_.create_publisher<example_interfaces::msg::Float64>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Float64 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/Int32") {
    av.publisher = node_.create_publisher<example_interfaces::msg::Int32>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Int32 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/UInt8") {
    av.publisher = node_.create_publisher<example_interfaces::msg::UInt8>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created UInt8 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/Bool") {
    av.publisher = node_.create_publisher<example_interfaces::msg::Bool>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Bool publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/ByteMultiArray") {
    av.publisher = node_.create_publisher<example_interfaces::msg::ByteMultiArray>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created ByteMultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "ur_dashboard_msgs/RobotMode") {
    av.publisher = node_.create_publisher<ur_dashboard_msgs::msg::RobotMode>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created RobotMode publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "ur_dashboard_msgs/SafetyMode") {
    av.publisher = node_.create_publisher<ur_dashboard_msgs::msg::SafetyMode>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created SafetyMode publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "builtin_interfaces/Time") {
    av.publisher = node_.create_publisher<builtin_interfaces::msg::Time>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Time publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "sensor_msgs/Temperature") {
    av.publisher = node_.create_publisher<sensor_msgs::msg::Temperature>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Temperature publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/Float64MultiArray") {
    av.publisher = node_.create_publisher<example_interfaces::msg::Float64MultiArray>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Float64MultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "example_interfaces/Int32MultiArray") {
    av.publisher = node_.create_publisher<example_interfaces::msg::Int32MultiArray>(av.config.topic, qos);
    RCLCPP_INFO(node_.get_logger(), "Created Int32MultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else {
    RCLCPP_WARN(node_.get_logger(), "Unknown output type: %s for variable %s", av.config.output_type.c_str(),
                av.name.c_str());
  }
}

void Publisher::publish(const urcl::rtde_interface::DataPackage& pkg)
{
  for (const auto& av : active_variables_) {
    try {
      publishVariable(av, pkg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 3000, "Error publishing %s: %s", av.name.c_str(),
                            e.what());
    }
  }
}

void Publisher::publishVariable(const ActiveVariable& av, const urcl::rtde_interface::DataPackage& pkg)
{
  try {
    if (av.config.rtde_type == "VECTOR6D") {
      std::array<double, 6> array_data;
      if (!pkg.getData(av.name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR6D data for " + av.config.name);
      }
      if (av.config.output_type == "geometry_msgs/TwistStamped") {
        auto msg = Converter::vectorToTwistStamped<6>(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/WrenchStamped") {
        auto msg = Converter::vector6dToWrenchStamped(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/PoseStamped") {
        auto msg = Converter::vector6dToPoseStamped(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/InertiaStamped") {
        auto msg = Converter::vector6dToInertiaStamped(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::InertiaStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/AccelStamped") {
        auto msg = Converter::vectorToAccelStamped<6>(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::AccelStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "example_interfaces/Float64MultiArray") {
        auto msg = Converter::vector6dToFloat64MultiArray(array_data);
        std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>>(av.publisher)
            ->publish(*msg);
      }
    } else if (av.config.rtde_type == "UINT32") {
      std::uint32_t uint32_data = 0;
      if (!pkg.getData(av.name, uint32_data)) {
        throw std::runtime_error("Failed to get UINT32 data for " + (av.name));
      }

      if (av.config.output_type == "example_interfaces/ByteMultiArray") {
        auto msg = Converter::uint32ToByteMultiArray(uint32_data);
        std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::ByteMultiArray>>(av.publisher)
            ->publish(*msg);
      } else if (av.config.output_type == "example_interfaces/UInt32") {
        auto msg = Converter::uint32ToMsg(uint32_data);
        std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::UInt32>>(av.publisher)->publish(*msg);
      }
    } else if (av.config.rtde_type == "INT32") {
      std::int32_t int32_data = 0;
      if (!pkg.getData(av.name, int32_data)) {
        throw std::runtime_error("Failed to get INT32 data for " + (av.name));
      }

      if (av.config.output_type == "example_interfaces/Int32") {
        auto msg = Converter::int32ToMsg(int32_data);
        std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::Int32>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "ur_dashboard_msgs/RobotMode") {
        auto msg = Converter::int32ToURRobotModeMsg(int32_data);
        std::static_pointer_cast<rclcpp::Publisher<ur_dashboard_msgs::msg::RobotMode>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "ur_dashboard_msgs/SafetyMode") {
        auto msg = Converter::int32ToURSafetyModeMsg(int32_data);
        std::static_pointer_cast<rclcpp::Publisher<ur_dashboard_msgs::msg::SafetyMode>>(av.publisher)->publish(*msg);
      }
    } else if (av.config.rtde_type == "UINT64") {
      std::uint64_t uint64_data = 0;
      if (!pkg.getData(av.name, uint64_data)) {
        throw std::runtime_error("Failed to get UINT64 data for " + (av.name));
      }
      auto msg = Converter::uint64ToByteMultiArray(uint64_data);
      std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::ByteMultiArray>>(av.publisher)->publish(*msg);
    } else if (av.config.rtde_type == "BOOL") {
      bool bool_data = 0;
      if (!pkg.getData(av.name, bool_data)) {
        throw std::runtime_error("Failed to get Bool data for " + (av.name));
      }
      auto msg = Converter::boolToMsg(bool_data);
      std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::Bool>>(av.publisher)->publish(*msg);
    } else if (av.config.rtde_type == "VECTOR6INT") {
      std::array<std::int32_t, 6> array_data;
      if (!pkg.getData(av.name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR6INT data for " + (av.name));
      }
      auto msg = Converter::vector6intToMsg(array_data);
      std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::Int32MultiArray>>(av.publisher)
          ->publish(*msg);
    } else if (av.config.rtde_type == "UINT8") {
      std::uint8_t uint8_data = 0;
      if (!pkg.getData(av.name, uint8_data)) {
        throw std::runtime_error("Failed to get UINT8 data for " + (av.name));
      }
      auto msg = Converter::uint8Tomsg(uint8_data);
      std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::UInt8>>(av.publisher)->publish(*msg);
    } else if (av.config.rtde_type == "DOUBLE") {
      std::double_t double_data = 0;
      if (!pkg.getData(av.name, double_data)) {
        throw std::runtime_error("Failed to get DOUBLE data for " + (av.name));
      }

      if (av.config.output_type == "example_interfaces/Float64") {
        auto msg = Converter::doubleToMsg(double_data);
        std::static_pointer_cast<rclcpp::Publisher<example_interfaces::msg::Float64>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "builtin_interfaces/Time") {
        auto msg = Converter::doubleToTimeMsg(double_data);
        std::static_pointer_cast<rclcpp::Publisher<builtin_interfaces::msg::Time>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "sensor_msgs/Temperature") {
        auto msg = Converter::doubleToTemperatureMsg(double_data);
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Temperature>>(av.publisher)->publish(*msg);
      }
    } else if (av.config.rtde_type == "VECTOR3D") {
      std::array<double, 3> array_data;
      if (!pkg.getData(av.name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR3D data for " + av.name);
      }
      if (av.config.output_type == "geometry_msgs/TwistStamped") {
        auto msg = Converter::vectorToTwistStamped<3>(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/Vector3Stamped") {
        auto msg = Converter::vector3dToVector3Stamped(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/PointStamped") {
        auto msg = Converter::vector3dToPointStamped(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::PointStamped>>(av.publisher)->publish(*msg);
      } else if (av.config.output_type == "geometry_msgs/AccelStamped") {
        auto msg = Converter::vectorToAccelStamped<3>(array_data);
        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::AccelStamped>>(av.publisher)->publish(*msg);
      }
    }
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to get/convert ") + (av.name + ": " + e.what()));
  }
}

const std::vector<std::string> Publisher::effective_keys() const
{
  std::vector<std::string> keys;
  for (const auto& av : active_variables_) {
    keys.push_back(av.name);
  }
  return keys;
}

}  // namespace ur_rtde_publisher