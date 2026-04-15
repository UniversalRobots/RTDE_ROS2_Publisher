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

#include "ur_rtde_publisher/rtde_publisher.hpp"

namespace ur_rtde_publisher
{
RTDEPublisher::RTDEPublisher(rclcpp::Node& node, const std::string& config_path, const std::string& tf_prefix)
  : node_(node)
{
  loadConfig(config_path, tf_prefix);
}

void RTDEPublisher::loadConfig(const std::string& config_path, const std::string& tf_prefix)
{
  YAML::Node config = YAML::LoadFile(config_path);
  all_variables_.clear();

  const auto& rtde_vars = config["rtde_variables"];

  for (auto group_it = rtde_vars.begin(); group_it != rtde_vars.end(); ++group_it) {
    std::string group_name = group_it->first.as<std::string>();
    const auto& group_config = group_it->second;

    std::string rtde_type;
    std::string output_type;
    YAML::Node variables;

    try {
      if (!group_config["rtde_type"]) {
        throw std::runtime_error("No RTDE type specified for group: " + group_name);
      }
      if (!group_config["output_type"]) {
        throw std::runtime_error("No output type specified for group: " + group_name);
      }
      if (!group_config["variables"] || !group_config["variables"].IsSequence()) {
        throw std::runtime_error("No variables array specified for group: " + group_name);
      }

      rtde_type = group_config["rtde_type"].as<std::string>();
      output_type = group_config["output_type"].as<std::string>();
      variables = group_config["variables"];

    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_.get_logger(), "%s", e.what());
      continue;
    }

    for (const auto& var_node : variables) {
      std::string var_pattern;
      std::string frame_id;

      if (var_node.IsMap()) {
        var_pattern = var_node["name"].as<std::string>();
        frame_id = var_node["frame_id"].as<std::string>();
      } else if (var_node.IsScalar()) {
        var_pattern = var_node.as<std::string>();
      } else {
        RCLCPP_ERROR(node_.get_logger(), "Unsupported variable format");
        continue;
      }
      auto expanded_vars = expandRange(var_pattern);

      for (const auto& var_name : expanded_vars) {
        auto [_, inserted] = seen_vars.insert(var_name);
        if (!inserted) {
          RCLCPP_WARN(node_.get_logger(), "Duplicate RTDE variable '%s' ignored", var_name.c_str());
          continue;
        }

        VariableConfig vc;
        vc.name = var_name;
        vc.rtde_type = rtde_type;
        vc.output_type = output_type;
        vc.topic = "rtde/" + var_name;

        if (!frame_id.empty())
          vc.frame_id = tf_prefix.empty() ? frame_id : tf_prefix + "/" + frame_id;

        all_variables_.push_back(vc);
      }
    }
  }

  RCLCPP_INFO(node_.get_logger(), "YAML file loaded successfully.");
}

std::vector<std::string> RTDEPublisher::expandRange(const std::string& pattern)
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
    RCLCPP_WARN(node_.get_logger(), "Failed to expand range pattern '%s', using as-is", pattern.c_str());
    return { pattern };
  }
}

void RTDEPublisher::createPublishersForRecipe(const std::vector<std::string>& recipe)
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

    if (!createPublisher(av)) {
      RCLCPP_WARN(node_.get_logger(), "Variable '%s' has no valid publisher, skipping", var_name.c_str());
      continue;
    }

    active_variables_.push_back(av);
  }
}

bool RTDEPublisher::createPublisher(ActiveVariable& av)
{
  rclcpp::QoS qos = rclcpp::SensorDataQoS();

  const std::string name = av.name;
  const std::string frame = av.config.frame_id;

  if (av.config.output_type == "geometry_msgs/TwistStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::TwistStamped>(av.config.topic, qos);

    if (av.config.rtde_type == "VECTOR3D") {
      av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
        std::array<double, 3> data;
        if (!pkg.getData(name, data)) {
          throw std::runtime_error("Failed to get VECTOR3D data for " + name);
        }
        pub->publish(RTDEConverter::vectorToTwistStamped<3>(data, frame, t));
      };

    } else if (av.config.rtde_type == "VECTOR6D") {
      av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
        std::array<double, 6> data;
        if (!pkg.getData(name, data)) {
          throw std::runtime_error("Failed to get VECTOR6D data for " + name);
        }
        pub->publish(RTDEConverter::vectorToTwistStamped<6>(data, frame, t));
      };

    } else {
      RCLCPP_WARN(node_.get_logger(), "Unsupported RTDE type '%s' for TwistStamped", av.config.rtde_type.c_str());
      return false;
    }

    RCLCPP_INFO(node_.get_logger(), "Created TwistStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/WrenchStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::WrenchStamped>(av.config.topic, qos);

    av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      std::array<double, 6> array_data;
      if (!pkg.getData(name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR6D data for " + name);
      }
      auto msg = RTDEConverter::vector6dToWrenchStamped(array_data, frame, t);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created WrenchStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/PoseStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::PoseStamped>(av.config.topic, qos);

    av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      std::array<double, 6> array_data;
      if (!pkg.getData(name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR6D data for " + name);
      }
      auto msg = RTDEConverter::vector6dToPoseStamped(array_data, frame, t);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created PoseStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/AccelStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::AccelStamped>(av.config.topic, qos);

    if (av.config.rtde_type == "VECTOR3D") {
      av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
        std::array<double, 3> array_data;
        if (!pkg.getData(name, array_data)) {
          throw std::runtime_error("Failed to get VECTOR3D data for " + name);
        }
        auto msg = RTDEConverter::vectorToAccelStamped<3>(array_data, frame, t);
        pub->publish(msg);
      };
    } else if (av.config.rtde_type == "VECTOR6D") {
      av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
        std::array<double, 6> array_data;
        if (!pkg.getData(name, array_data)) {
          throw std::runtime_error("Failed to get VECTOR6D data for " + name);
        }
        auto msg = RTDEConverter::vectorToAccelStamped<6>(array_data, frame, t);
        pub->publish(msg);
      };
    } else {
      RCLCPP_WARN(node_.get_logger(), "Unsupported RTDE type '%s' for TwistStamped", av.config.rtde_type.c_str());
      return false;
    }

    RCLCPP_INFO(node_.get_logger(), "Created AccelStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/PointStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::PointStamped>(av.config.topic, qos);

    av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      std::array<double, 3> array_data;
      if (!pkg.getData(name, array_data)) {
        throw std::runtime_error("Failed to get VECTOR3D data for " + name);
      }
      auto msg = RTDEConverter::vector3dToPointStamped(array_data, frame, t);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created PointStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/Vector3Stamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::Vector3Stamped>(av.config.topic, qos);

    av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      std::array<double, 3> data;
      if (!pkg.getData(name, data)) {
        throw std::runtime_error("Failed to get VECTOR3D data for " + name);
      }
      auto msg = RTDEConverter::vector3dToVector3Stamped(data, frame, t);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Vector3Stamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "geometry_msgs/InertiaStamped") {
    auto pub = node_.create_publisher<geometry_msgs::msg::InertiaStamped>(av.config.topic, qos);

    av.publish = [pub, name, frame](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      std::array<double, 6> data;
      if (!pkg.getData(name, data)) {
        throw std::runtime_error("Failed to get VECTOR6D data for " + name);
      }
      auto msg = RTDEConverter::vector6dToInertiaStamped(data, frame, t);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created InertiaStamped publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/UInt32") {
    auto pub = node_.create_publisher<example_interfaces::msg::UInt32>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      uint32_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get UINT32 data for " + name);
      }
      example_interfaces::msg::UInt32 msg;
      msg.data = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created UInt32 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/Float64") {
    auto pub = node_.create_publisher<example_interfaces::msg::Float64>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      double value = 0.0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get DOUBLE data for " + name);
      }
      example_interfaces::msg::Float64 msg;
      msg.data = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Float64 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/Int32") {
    auto pub = node_.create_publisher<example_interfaces::msg::Int32>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      int32_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get INT32 data for " + name);
      }
      example_interfaces::msg::Int32 msg;
      msg.data = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Int32 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/UInt8") {
    auto pub = node_.create_publisher<example_interfaces::msg::UInt8>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      uint8_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get UINT8 data for " + name);
      }
      example_interfaces::msg::UInt8 msg;
      msg.data = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created UInt8 publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/Bool") {
    auto pub = node_.create_publisher<example_interfaces::msg::Bool>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      bool value = false;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get BOOL data for " + name);
      }
      example_interfaces::msg::Bool msg;
      msg.data = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Bool publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/ByteMultiArray") {
    if (av.config.rtde_type == "UINT32") {
      auto pub = node_.create_publisher<example_interfaces::msg::ByteMultiArray>(av.config.topic, qos);

      av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
        uint32_t value = 0;
        if (!pkg.getData(name, value)) {
          throw std::runtime_error("Failed to get UINT32 data for " + name);
        }
        auto msg = RTDEConverter::uint32ToByteMultiArray(value);
        pub->publish(msg);
      };
    } else if (av.config.rtde_type == "UINT64") {
      auto pub = node_.create_publisher<example_interfaces::msg::ByteMultiArray>(av.config.topic, qos);

      av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
        uint64_t value = 0;
        if (!pkg.getData(name, value)) {
          throw std::runtime_error("Failed to get UINT64 data for " + name);
        }
        auto msg = RTDEConverter::uint64ToByteMultiArray(value);
        pub->publish(msg);
      };
    }

    RCLCPP_INFO(node_.get_logger(), "Created ByteMultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "ur_dashboard_msgs/RobotMode") {
    auto pub = node_.create_publisher<ur_dashboard_msgs::msg::RobotMode>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      int32_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get INT32 data for " + name);
      }
      ur_dashboard_msgs::msg::RobotMode msg;
      msg.mode = static_cast<int8_t>(value);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created RobotMode publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());
  } else if (av.config.output_type == "ur_dashboard_msgs/SafetyStatus") {
    auto pub = node_.create_publisher<ur_dashboard_msgs::msg::SafetyStatus>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      int32_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get INT32 data for " + name);
      }
      ur_dashboard_msgs::msg::SafetyStatus msg;
      msg.status = static_cast<int8_t>(value);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created SafetyStatus publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "ur_dashboard_msgs/SafetyMode") {
    auto pub = node_.create_publisher<ur_dashboard_msgs::msg::SafetyMode>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      int32_t value = 0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get INT32 data for " + name);
      }
      ur_dashboard_msgs::msg::SafetyMode msg;
      msg.mode = static_cast<uint8_t>(value);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created SafetyMode publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "builtin_interfaces/Time") {
    auto pub = node_.create_publisher<builtin_interfaces::msg::Time>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      double value = 0.0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get DOUBLE data for " + name);
      }
      builtin_interfaces::msg::Time msg;
      msg.sec = static_cast<int32_t>(value);
      msg.nanosec = static_cast<uint32_t>((value - msg.sec) * 1e9);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Time publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "sensor_msgs/Temperature") {
    auto pub = node_.create_publisher<sensor_msgs::msg::Temperature>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time& t) {
      double value = 0.0;
      if (!pkg.getData(name, value)) {
        throw std::runtime_error("Failed to get DOUBLE data for " + name);
      }
      sensor_msgs::msg::Temperature msg;
      msg.header.stamp = t;
      msg.temperature = value;
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Temperature publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/Float64MultiArray") {
    auto pub = node_.create_publisher<example_interfaces::msg::Float64MultiArray>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      std::array<double, 6> data;
      if (!pkg.getData(name, data)) {
        throw std::runtime_error("Failed to get VECTOR6D data for " + name);
      }
      auto msg = RTDEConverter::vector6dToFloat64MultiArray(data);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Float64MultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else if (av.config.output_type == "example_interfaces/Int32MultiArray") {
    auto pub = node_.create_publisher<example_interfaces::msg::Int32MultiArray>(av.config.topic, qos);

    av.publish = [pub, name](const urcl::rtde_interface::DataPackage& pkg, const rclcpp::Time&) {
      std::array<int32_t, 6> data;
      if (!pkg.getData(name, data)) {
        throw std::runtime_error("Failed to get VECTOR6INT data for " + name);
      }
      auto msg = RTDEConverter::vector6intToMsg(data);
      pub->publish(msg);
    };

    RCLCPP_INFO(node_.get_logger(), "Created Int32MultiArray publisher for %s on topic %s", av.name.c_str(),
                av.config.topic.c_str());

  } else {
    RCLCPP_WARN(node_.get_logger(), "Unknown output type: %s for variable %s", av.config.output_type.c_str(),
                av.name.c_str());
    return false;
  }

  return true;
}

void RTDEPublisher::publish(const urcl::rtde_interface::DataPackage& pkg, rclcpp::Time& packet_time)
{
  for (const auto& av : active_variables_) {
    try {
      av.publish(pkg, packet_time);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 3000, "Error publishing %s: %s", av.name.c_str(),
                            e.what());
    }
  }
}

const std::vector<std::string> RTDEPublisher::effective_keys() const
{
  std::vector<std::string> keys;
  for (const auto& av : active_variables_) {
    keys.push_back(av.name);
  }
  return keys;
}

}  // namespace ur_rtde_publisher