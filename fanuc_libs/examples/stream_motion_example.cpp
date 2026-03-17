// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

#include "stream_motion/stream.hpp"
#include "rmi/rmi.hpp"

void printStatus(const stream_motion::RobotStatusPacket& status)
{
  std::cout << "packet_type: " << status.packet_type << std::endl;
  std::cout << "version_no: " << status.version_no << std::endl;
  std::cout << "sequence_no: " << status.sequence_no << std::endl;
  std::cout << "status: " << static_cast<int>(status.status) << std::endl;
  std::cout << "robot_status: " << static_cast<int>(status.robot_status) << std::endl;
  std::cout << "contact_stop_status: " << static_cast<int>(status.contact_stop_status) << std::endl;
  std::cout << "time_stamp: " << status.time_stamp << std::endl;
  for (int i = 0; i < status.position.size(); ++i)
  {
    std::cout << "position[" << i << "]: " << status.position[i] << std::endl;
  }
  for (int i = 0; i < status.joint_angle.size(); ++i)
  {
    std::cout << "joint_angle[" << i << "]: " << status.joint_angle[i] << std::endl;
  }
  for (int i = 0; i < status.current.size(); ++i)
  {
    std::cout << "current[" << i << "]: " << status.current[i] << std::endl;
  }
  std::cout << "status.io_status[1]: " << (int)status.io_status[1] << std::endl;
}

void printJointLimits(const stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      const stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      const stream_motion::RobotThresholdPacket& robot_threshold_jerk)
{
  std::cout << "Velocity limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_velocity.no_payload[i] << std::endl;
  }
  std::cout << "Acceleration limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_acceleration.no_payload[i] << std::endl;
  }
  std::cout << "Jerk limits:" << std::endl;
  for (size_t i = 0; i < 20; ++i)
  {
    std::cout << "  joint[" << i << "]: " << robot_threshold_jerk.no_payload[i] << std::endl;
  }
}
int main()
{
  std::string robot_ip = "10.84.0.60";

  // Bootstrap RMI
  std::cout << "Bootstrapping Stream Motion via RMI." << std::endl;
  rmi::RMIConnection rmi_connection(robot_ip);
  rmi_connection.connect(5.0);
  try
  {
    rmi_connection.getStatus(std::nullopt);
    rmi_connection.reset(std::nullopt);
    rmi_connection.initializeRemoteMotion(std::nullopt);
  }
  catch (const std::runtime_error&)
  {
    std::cout << "Need to reset and abort before initialization" << std::endl;
    rmi_connection.abort(std::nullopt);
    rmi_connection.reset(std::nullopt);
    rmi_connection.getStatus(std::nullopt);
    rmi_connection.initializeRemoteMotion(std::nullopt);
  }

  const auto program_call = rmi_connection.programCallNonBlocking("STREAM_MOTN");
  std::cout << "Requested RMI program call for STREAM_MOTN with sequence_id=" << program_call.SequenceID << std::endl;
  
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  // Crete IO config for reading and writing
  std::array<uint8_t, 256> io_command{};
  for (int i = 0; i < 256; ++i)
  {
    io_command[i] = 0xFF;
  }

  std::array<stream_motion::GPIOControlConfig, 32> gpio_config{};
  gpio_config[0].command_type = stream_motion::GPIOCommandType::IOCmd;
  gpio_config[0].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[0].start = 1;
  gpio_config[0].length = 32;

  gpio_config[1].command_type = stream_motion::GPIOCommandType::IOState;
  gpio_config[1].gpio_type = static_cast<uint32_t>(stream_motion::IOType::F);
  gpio_config[1].start = 1;
  gpio_config[1].length = 32;

  stream_motion::StreamMotionConnection connection(robot_ip);
  
  // Get capability
  stream_motion::ControllerCapabilityResultPacket capability{};
  if (!connection.getControllerCapability(capability))
  {
    std::cerr << "Failed to get controller capability" << std::endl;
    return 1;
  }

  stream_motion::RobotThresholdPacket robot_threshold_velocity;
  stream_motion::RobotThresholdPacket robot_threshold_acceleration;
  stream_motion::RobotThresholdPacket robot_threshold_jerk;

  // Get the robot limits for axis 1 and print them out
  connection.getRobotLimits(1, robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);
  printJointLimits(robot_threshold_velocity, robot_threshold_acceleration, robot_threshold_jerk);

  // Setup GPIO
  connection.configureGPIO(gpio_config);

  // Start the streaming protocol
  connection.sendStartPacket();
  stream_motion::RobotStatusPacket status{};
  
  // Wait for first status packet
  bool got_status = false;
  for (int i = 0; i < 100; ++i) {
    if (connection.getStatusPacket(status)) {
      got_status = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  if (!got_status) {
    std::cerr << "Failed to receive initial status packet" << std::endl;
    return 1;
  }

  // Print the initial status packet
  printStatus(status);

  std::array<double, stream_motion::kMaxAxisNumber> initial_command{};
  std::transform(status.joint_angle.begin(), status.joint_angle.end(), initial_command.begin(),
                 [](const float angle) { return static_cast<double>(angle); });

  // Hold position for 10 seconds
  constexpr int duration_ms = 10000;
  int sampling_rate = capability.sampling_rate > 0 ? capability.sampling_rate : 8;
  int num_steps = duration_ms / sampling_rate;
  
  std::cout << "Holding position for " << duration_ms << "ms (" << num_steps << " steps at " << sampling_rate << "ms)" << std::endl;

  for (int i = 0; i < num_steps; ++i)
  {
    if (!connection.getStatusPacket(status))
    {
      std::cerr << "Failed to get status packet at step " << i << std::endl;
      break;
    }
    
    bool is_last = (i == num_steps - 1);
    connection.sendCommand(initial_command, is_last, io_command);
  }

  // checking motion completed
  bool wait_for_completed = true;
  int timeout_count = 0;
  while (wait_for_completed && timeout_count < 100)
  {
    if (connection.getStatusPacket(status)) {
      wait_for_completed = (status.status & 1) || (status.status & 8);
    } else {
      timeout_count++;
    }
  }

  // Terminate the streaming protocol
  connection.sendStopPacket();

  // Print the final status packet
  printStatus(status);
  
  return 0;
}
