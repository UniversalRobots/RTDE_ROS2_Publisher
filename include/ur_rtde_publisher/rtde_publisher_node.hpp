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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <ur_client_library/rtde/rtde_client.h>
#include "ur_rtde_publisher/rtde_publisher.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <ur_client_library/log.h>
#include <ur_client_library/types.h>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <filesystem>

namespace ur_rtde_publisher
{
/**
 * @brief ROS 2 node that bridges UR RTDE data to ROS topics.
 */
class RtdePublisherNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs the node, declares parameters, loads config, creates publishers,
   *        and starts RTDE communication (which runs in a background thread).
   */
  RtdePublisherNode();

  /**
   * @brief Constructs the node using the provided NodeOptions.
   */
  explicit RtdePublisherNode(const rclcpp::NodeOptions& options);

  /**
   * @brief Destructor of RtdePublisherNode.
   * It reads the last data from the threat before removing the object.
   */
  ~RtdePublisherNode();

  // No-copyable
  RtdePublisherNode(const RtdePublisherNode&) = delete;
  RtdePublisherNode& operator=(const RtdePublisherNode&) = delete;

  /**
   * @brief Configure the node.
   * @return true if configuration succeeds, false otherwise.
   */
  bool configure();

  /**
   * @brief Load and validate the parameters used by the node.
   * @return true if all parameters were successfully loaded and validated; false otherwise.
   */
  bool loadParameters();

  /**
   * @brief Initialize the publishers used by the node.
   * @return true if the publishers were successfully initialized; false otherwise.
   */
  bool initPublisher();

  /**
   * @brief Initialize and start the UR RTDE client.
   * @return true if initialization and start succeeded; false otherwise.
   */
  bool startRtde();

  /**
   * @brief Perform one RTDE cycle, fetch a data package and publish active variables.
   */
  void spinOnce();

private:
  /// @brief Helper that loads YAML mapping and creates ROS publishers per RTDE variable/group.
  std::unique_ptr<RTDEPublisher> rtde_publisher_;

  /// @brief Recipe (list of RTDE output keys) requested to the robot and published by this node.
  std::vector<std::string> output_recipe_;

  /// @brief Robot controller IP address (declared/read as parameter).
  std::string robot_ip_;

  /// @brief RTDE frequency for getting data.
  int rtde_frequency_;

  /// @brief TF prefix for the frame IDs published.
  std::string tf_prefix_;

  /// @brief Keys actually created and active after validating with the YAML mapping.
  std::vector<std::string> effective_keys_;

  /// @brief Owned UR RTDE client.
  std::unique_ptr<urcl::rtde_interface::RTDEClient> rtde_client_;

  /// @brief URCL notifier for logging/callback hooks.
  urcl::comm::INotifier notifier_{};

  /// @brief Reusable buffer holding the latest RTDE data package.
  std::unique_ptr<urcl::rtde_interface::DataPackage> pkg_;

  /// @brief Flag to signal the read thread to stop.
  std::atomic<bool> stop_reading_{ false };

  /// @brief Background thread that reads RTDE data when available.
  std::thread read_thread_;

  /// @brief Worker function for the background read thread.
  void readThreadWorker();
};

}  // namespace ur_rtde_publisher