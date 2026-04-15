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

#include "ur_rtde_publisher/rtde_publisher_node.hpp"

using namespace std::chrono_literals;

namespace ur_rtde_publisher
{

RtdePublisherNode::RtdePublisherNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("ur_rtde_publisher_node", options)
{
}

RtdePublisherNode::RtdePublisherNode() : rclcpp::Node("ur_rtde_publisher_node")
{
}

RtdePublisherNode::~RtdePublisherNode()
{
  stop_reading_ = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool RtdePublisherNode::configure()
{
  return loadParameters() && initPublisher();
}

bool RtdePublisherNode::loadParameters()
{
  try {
    // Declare parameters
    output_recipe_ = declare_parameter<std::vector<std::string>>("output_recipe");
    robot_ip_ = declare_parameter<std::string>("robot_ip");
    rtde_frequency_ = declare_parameter<int>("rtde_frequency", 500);
    tf_prefix_ = declare_parameter<std::string>("tf_prefix", "");

    // Verify parameters
    if (output_recipe_.empty() || robot_ip_.empty()) {
      std::ostringstream oss;

      oss << "\033[1;31m"
          << "\n"
          << "ERROR: Missing required parameter(s): robot_ip and/or output_recipe.\n"
          << "Provide them like this:\n"
          << "  ros2 launch ur_rtde_publisher rtde_publisher.launch.xml \\\n"
          << "    robot_ip:=\"172.17.0.3\" \\\n"
          << "    output_recipe:=\"[\"payload\",\"timestamp\",\"safety_status\"]\" \033[0m \n";

      RCLCPP_ERROR(this->get_logger(), "%s", oss.str().c_str());
      return false;
    }

    if (rtde_frequency_ <= 0 || rtde_frequency_ > 500) {
      RCLCPP_ERROR(get_logger(), "Invalid rtde_frequency: %d Hz. Supported RTDE range is 1–500 Hz.", rtde_frequency_);
      return false;
    }

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load the parameters: %s", e.what());
    return false;
  }
}

bool RtdePublisherNode::initPublisher()
{
  try {
    // Load publisher config
    std::filesystem::path package_share_dir;
    ament_index_cpp::get_package_share_directory("ur_rtde_publisher", package_share_dir);

    std::filesystem::path config_path = package_share_dir.string() + "/config/rtde_map.yaml";

    // Initialize publisher
    rtde_publisher_ = std::make_unique<RTDEPublisher>(*this, config_path.string(), tf_prefix_);

    // Create publishers based on output_recipe and config
    rtde_publisher_->createPublishersForRecipe(output_recipe_);
    effective_keys_ = rtde_publisher_->effective_keys();

    if (effective_keys_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid RTDE keys after applying recipe and mapping");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Created %zu publishers from recipe", effective_keys_.size());
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize publisher: %s", e.what());
    return false;
  }
}

bool RtdePublisherNode::startRtde()
{
  try {
    // Initialize RTDE client
    rtde_client_ = std::make_unique<urcl::rtde_interface::RTDEClient>(robot_ip_, notifier_, effective_keys_,
                                                                      std::vector<std::string>{}, rtde_frequency_);
    rtde_client_->init();
    rtde_client_->start();

    RCLCPP_INFO(get_logger(), "RTDE client started (ip=%s) with %zu keys.", robot_ip_.c_str(), effective_keys_.size());

    // Initialize thread to keep reading RTDE data
    stop_reading_ = false;
    read_thread_ = std::thread(&RtdePublisherNode::readThreadWorker, this);
    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to start RTDE client: %s", e.what());
    rtde_client_.reset();
    return false;
  }
}

void RtdePublisherNode::readThreadWorker()
{
  RCLCPP_INFO(this->get_logger(), "RTDE read thread started");

  while (!stop_reading_) {
    try {
      if (rtde_client_) {
        spinOnce();
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Exception in RTDE read thread: %s",
                            e.what());
    }
  }

  RCLCPP_INFO(this->get_logger(), "RTDE read thread stopped");
}

void RtdePublisherNode::spinOnce()
{
  if (!rtde_client_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RTDE client not initialized. Skipping cycle.");
    return;
  }

  if (!pkg_)
    pkg_ = std::make_unique<urcl::rtde_interface::DataPackage>(effective_keys_);

  if (!rtde_client_->getDataPackage(*pkg_, 200ms)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No RTDE package received (timeout).");
    return;
  }

  rclcpp::Time packet_time = this->now();

  try {
    rtde_publisher_->publish(*pkg_, packet_time);

  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Publisher failed to publish RTDE package: %s",
                          e.what());
  }
}

}  // namespace ur_rtde_publisher
