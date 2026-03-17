// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "stream_motion/stream.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sockpp/inet_address.h"
#include "sockpp/udp_socket.h"
#include "stream_motion/byte_ops.hpp"
#include "stream_motion/packets.hpp"

namespace stream_motion
{
namespace
{
constexpr uint16_t kCommandPacketUnused = 0xFFFF;
constexpr int kThresholdPayloadLength = 20;
constexpr size_t kR50ControllerCapabilityResponseSize = 25;

bool ShouldLogDiscard(uint32_t& discard_count)
{
  ++discard_count;
  return discard_count <= 3 || (discard_count % 50) == 0;
}

std::string FormatByteString(const void* data, const size_t size, const size_t max_bytes = 32)
{
  const auto* bytes = static_cast<const uint8_t*>(data);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  const size_t bytes_to_print = std::min(size, max_bytes);
  for (size_t i = 0; i < bytes_to_print; ++i)
  {
    if (i > 0)
    {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned>(bytes[i]);
  }
  if (size > max_bytes)
  {
    stream << " ...";
  }
  return stream.str();
}

bool IsReadGPIOConfig(const GPIOControlConfig& config)
{
  return config.command_type == GPIOCommandType::IOState || config.command_type == GPIOCommandType::NumRegState;
}

bool IsWriteGPIOConfig(const GPIOControlConfig& config)
{
  return config.command_type == GPIOCommandType::IOCmd || config.command_type == GPIOCommandType::NumRegCmd;
}

void CheckOverlapping(const std::vector<GPIOControlConfig>& gpio_config)
{
  std::unordered_map<uint64_t, std::vector<std::reference_wrapper<const GPIOControlConfig>>> configs_by_type;
  for (const auto& config : gpio_config)
  {
    const uint64_t hash = (static_cast<uint64_t>(config.command_type) << 32) | config.gpio_type;
    configs_by_type[hash].push_back(config);
  }

  for (auto& [_, sorted_gpio_config] : configs_by_type)
  {
    std::sort(sorted_gpio_config.begin(), sorted_gpio_config.end(),
              [](const GPIOControlConfig& a, const GPIOControlConfig& b) { return a.start < b.start; });
    for (size_t i = 0; i + 1 < sorted_gpio_config.size(); ++i)
    {
      const auto& current = sorted_gpio_config[i];
      if (const auto& next = sorted_gpio_config[i + 1]; current.get().start + current.get().length > next.get().start)
      {
        throw std::invalid_argument("The GPIO configurations has overlapping ranges which is not supported.");
      }
    }
  }
}

// Returns the number of GPIO values that can be packed in 4 bytes based on their type.
int32_t CalculateNumPackedValues(const GPIOControlConfig& gpio_config)
{
  switch (gpio_config.command_type)
  {
    case GPIOCommandType::IOState:
      [[fallthrough]];
    case GPIOCommandType::IOCmd:
    {
      if (gpio_config.gpio_type == static_cast<uint32_t>(IOType::AO) ||
          gpio_config.gpio_type == static_cast<uint32_t>(IOType::AI))
      {
        return 2;
      }
      return 32;
    }
    case GPIOCommandType::NumRegState:
      [[fallthrough]];
    case GPIOCommandType::NumRegCmd:
      return 1;  // For numeric registers, we assume a float size of 4 bytes
    default:
      return 0;  // No bits for None command type
  }
}

// Returns the number of bytes needed for a config with 4 byte alignment.
int32_t CalculateNumBytesConfig(const GPIOControlConfig& gpio_config)
{
  const int32_t denominator = CalculateNumPackedValues(gpio_config);
  if (denominator == 0)
  {
    return 0;
  }
  return std::max(static_cast<int32_t>(1 + (gpio_config.length - 1) / denominator) * 4, 4);
}
}  // namespace

struct StreamMotionConnection::PSocketImpl
{
  PSocketImpl(const std::string& robot_ip_address, const uint16_t robot_port, const double timeout)
    : server_address{ robot_ip_address, robot_port }, timeout{ timeout }
  {
    sock.connect(server_address);
    sock.set_non_blocking(true);
    std::cout << "Created UDP socket at: " << sock.address() << std::endl;
  }

  bool sendStopPacketForVersion(uint32_t version)
  {
    StopPacket stop_packet{};
    stop_packet.packet_type = swapBytesIfNeeded(stop_packet.packet_type);
    stop_packet.version_no = swapBytesIfNeeded(version);
    return send(stop_packet);
  }

  void drainSocket(std::chrono::milliseconds quiet_period)
  {
    std::array<uint8_t, 512> buffer{};
    const auto deadline = std::chrono::steady_clock::now() + quiet_period;
    while (std::chrono::steady_clock::now() < deadline)
    {
      auto res = sock.recv(buffer.data(), buffer.size());
      if (res && res.value() > 0)
      {
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  template <typename T>
  bool send(const T& value)
  {
    const auto buf = reinterpret_cast<const void*>(&value);
    constexpr size_t kPacketNumBytes = sizeof(T);

    if (const auto res = sock.send(buf, kPacketNumBytes); res != kPacketNumBytes)
    {
      std::cerr << "Error writing to the UDP socket: " << res.error_message() << std::endl;
      return false;
    }
    return true;
  }

  template <typename T>
  bool receive(T& value)
  {
    value = T();
    std::array<uint8_t, 512> raw_bytes{};
    const auto start_time = std::chrono::steady_clock::now();
    uint32_t discard_count = 0;
    while (true)
    {
      constexpr size_t kPacketNumBytes = sizeof(T);
      sockpp::result<size_t> res = sock.recv(raw_bytes.data(), raw_bytes.size());
      const bool has_value = static_cast<bool>(res);
      const size_t received_size = has_value ? res.value() : 0;
      if (received_size == kPacketNumBytes)
      {
        std::memcpy(&value, raw_bytes.data(), kPacketNumBytes);
        return true;
      }
      if (has_value && received_size > 0)
      {
        if (ShouldLogDiscard(discard_count))
        {
          std::cerr << "Discarding unexpected UDP packet while waiting for " << kPacketNumBytes
                    << "-byte response. got=" << received_size << " from " << server_address
                    << " raw=" << FormatByteString(raw_bytes.data(), received_size) << std::endl;
        }
        continue;
      }
      if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(timeout))
      {
        std::cerr << "Timeout while reading from UDP socket. Expected " << kPacketNumBytes << " bytes from "
                  << server_address << ". Last socket error: " << res.error_message() << std::endl;
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  bool receiveControllerCapability(ControllerCapabilityResultPacket& controller_capability)
  {
    controller_capability = ControllerCapabilityResultPacket{};
    std::array<uint8_t, 512> raw_bytes{};
    const auto start_time = std::chrono::steady_clock::now();
    uint32_t discard_count = 0;
    while (true)
    {
      sockpp::result<size_t> res = sock.recv(raw_bytes.data(), raw_bytes.size());
      const bool has_value = static_cast<bool>(res);
      const size_t received_size = has_value ? res.value() : 0;
      if (received_size == sizeof(ControllerCapabilityResultPacket) || received_size == kR50ControllerCapabilityResponseSize)
      {
        std::memcpy(&controller_capability, raw_bytes.data(), received_size);
        if (received_size == kR50ControllerCapabilityResponseSize)
        {
          controller_capability.rob_status_use_tcp = raw_bytes[24];
        }
        ControllerCapabilityResultPacket swapped = controller_capability;
        swapControllerCapabilityResponseBytes(swapped);
        if (swapped.packet_type != kGetCapabilityPacketType)
        {
          if (ShouldLogDiscard(discard_count))
          {
            std::cerr << "Discarding unexpected packet while waiting for controller capability. size="
                      << received_size << " packet_type=" << swapped.packet_type
                      << " raw=" << FormatByteString(raw_bytes.data(), received_size) << std::endl;
          }
          continue;
        }
        if (received_size == kR50ControllerCapabilityResponseSize)
        {
          std::cerr << "Received 25-byte controller capability response from " << server_address
                    << ". Accepting it using R-50 compatibility parsing. Raw bytes: "
                    << FormatByteString(raw_bytes.data(), received_size) << std::endl;
        }
        return true;
      }
      if (has_value && received_size > 0)
      {
        if (ShouldLogDiscard(discard_count))
        {
          std::cerr << "Discarding unexpected UDP packet while waiting for controller capability. Expected "
                    << sizeof(ControllerCapabilityResultPacket) << " or " << kR50ControllerCapabilityResponseSize
                    << " bytes, got " << received_size << " bytes from " << server_address
                    << ". Raw bytes: " << FormatByteString(raw_bytes.data(), received_size) << std::endl;
        }
        continue;
      }
      if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(timeout))
      {
        std::cerr << "Timeout while reading controller capability from UDP socket. Expected "
                  << sizeof(ControllerCapabilityResultPacket) << " or " << kR50ControllerCapabilityResponseSize
                  << " bytes from " << server_address << ". Last socket error: " << res.error_message() << std::endl;
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  sockpp::udp_socket sock;
  sockpp::inet_address server_address;
  double timeout;
};

StreamMotionConnection::~StreamMotionConnection() = default;

StreamMotionConnection::StreamMotionConnection(const std::string& robot_ip_address, const double timeout,
                                               uint16_t robot_port)
  : StreamMotionInterface(), socket_impl_{ std::make_unique<PSocketImpl>(robot_ip_address, robot_port, timeout) }
{
}

bool StreamMotionConnection::getRobotLimits(const uint32_t axis_number, RobotThresholdPacket& robot_threshold_velocity,
                                            RobotThresholdPacket& robot_threshold_acceleration,
                                            RobotThresholdPacket& robot_threshold_jerk) const
{
  if (axis_number < 1 || axis_number > kMaxAxisNumber)
  {
    throw std::out_of_range("Axis number must be between 1 and 9.");
  }
  ThresholdPacket threshold_packet{};
  threshold_packet.packet_type = swapBytesIfNeeded(threshold_packet.packet_type);
  threshold_packet.version_no = swapBytesIfNeeded(version_no_);
  threshold_packet.axis_number = swapBytesIfNeeded(axis_number);
  // Velocity limits
  threshold_packet.threshold_type = swapBytesIfNeeded(0);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_velocity))
  {
    std::cerr << "Failed to receive velocity threshold response for axis " << axis_number << '.' << std::endl;
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_velocity);
  // Acceleration limits
  threshold_packet.threshold_type = swapBytesIfNeeded(1);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_acceleration))
  {
    std::cerr << "Failed to receive acceleration threshold response for axis " << axis_number << '.' << std::endl;
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_acceleration);
  // Jerk limits
  threshold_packet.threshold_type = swapBytesIfNeeded(2);
  socket_impl_->send(threshold_packet);
  if (!socket_impl_->receive(robot_threshold_jerk))
  {
    std::cerr << "Failed to receive jerk threshold response for axis " << axis_number << '.' << std::endl;
    return false;
  }
  swapRobotThresholdPacketBytes(robot_threshold_jerk);

  return true;
}

bool StreamMotionConnection::configureGPIO(const GPIOConfiguration& config) const
{
  ValidateGPIOConfig(config);

  GPIOConfigPacket gpio_config_packet{};
  gpio_config_packet.version_no = version_no_;
  gpio_config_packet.gpio_configuration = config;
  swapGPIOConfigPacketBytes(gpio_config_packet);

  socket_impl_->send(gpio_config_packet);

  GPIOConfigResultPacket gpio_config_result_packet{};
  if (!socket_impl_->receive(gpio_config_result_packet))
  {
    std::cerr << "Failed to get response from IO configuration." << std::endl;
    return false;
  }
  gpio_config_result_packet.packet_type = swapBytesIfNeeded(gpio_config_result_packet.packet_type);
  gpio_config_result_packet.result = swapBytesIfNeeded(gpio_config_result_packet.result);
  gpio_config_result_packet.ptf = swapBytesIfNeeded(gpio_config_result_packet.ptf);
  if (gpio_config_result_packet.result != 0)
  {
    std::cerr << "IO configuration failed with error code: " << gpio_config_result_packet.result << std::endl;
    return false;
  }
  return true;
}

bool StreamMotionConnection::getControllerCapability(ControllerCapabilityResultPacket& controller_capability)
{
  // A previously interrupted session can leave the controller sending legacy
  // status packets to the next source port that talks to Stream Motion.
  // Send a few stop packets and drain any queued datagrams before negotiating capability.
  socket_impl_->sendStopPacketForVersion(1);
  socket_impl_->sendStopPacketForVersion(2);
  socket_impl_->sendStopPacketForVersion(3);
  socket_impl_->drainSocket(std::chrono::milliseconds(50));

  ControllerCapabilityPacket controller_capability_packet{};
  controller_capability_packet.packet_type = kGetCapabilityPacketType;
  controller_capability_packet.version_no = version_no_;
  swapControllerCapabilityBytes(controller_capability_packet);
  socket_impl_->send(controller_capability_packet);
  controller_capability = ControllerCapabilityResultPacket{};
  if (!socket_impl_->receiveControllerCapability(controller_capability))
  {
    std::cerr << "Failed to get response for controller capability." << std::endl;
    return false;
  }
  swapControllerCapabilityResponseBytes(controller_capability);
  if (controller_capability.packet_type != kGetCapabilityPacketType)
  {
    std::cerr << "Unexpected controller capability packet type: " << controller_capability.packet_type
              << " (expected " << kGetCapabilityPacketType << ")." << std::endl;
  }
  std::cout << "Controller capability response: sampling_rate=" << controller_capability.sampling_rate
            << "ms, start_move=" << controller_capability.start_move
            << ", available_version=" << controller_capability.available_version
            << ", rob_status_use_tcp=" << controller_capability.rob_status_use_tcp << std::endl;
  version_no_ = controller_capability.available_version;

  return true;
}

void StreamMotionConnection::sendStartPacket() const
{
  std::cout << "[StreamMotion] Sending StartPacket with version_no=" << version_no_ << std::endl;
  if (version_no_ <= 2)
  {
    LegacyStartPacket start_packet{};
    start_packet.packet_type = swapBytesIfNeeded(start_packet.packet_type);
    start_packet.version_no = swapBytesIfNeeded(version_no_);
    socket_impl_->send(start_packet);
  }
  else
  {
    StartPacket start_packet{};
    start_packet.packet_type = swapBytesIfNeeded(start_packet.packet_type);
    start_packet.version_no = swapBytesIfNeeded(version_no_);
    socket_impl_->send(start_packet);
  }
}

void StreamMotionConnection::sendStopPacket() const
{
  std::cout << "[StreamMotion] Sending StopPacket with version_no=" << version_no_ << std::endl;
  StopPacket stop_packet{};
  stop_packet.packet_type = swapBytesIfNeeded(stop_packet.packet_type);
  stop_packet.version_no = swapBytesIfNeeded(version_no_);
  socket_impl_->send(stop_packet);
}

void StreamMotionConnection::configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const
{
  // Skip if client version smaller than 4 to keep backward compatibility
  // Otherwise HOST-380 System error 0x19,0x0 will be posted due to unknown packet type
  if (version_no_ >= 4)
  {
    ForceSensorConfigPacket force_sensor_config_packet{};
    force_sensor_config_packet.packet_type = swapBytesIfNeeded(force_sensor_config_packet.packet_type);
    force_sensor_config_packet.version_no = swapBytesIfNeeded(version_no_);
    force_sensor_config_packet.do_reset = swapBytesIfNeeded(do_reset);
    force_sensor_config_packet.fs_type = swapBytesIfNeeded(force_sensor_type);
    socket_impl_->send(force_sensor_config_packet);
  }
}

void swapCommandPacketBytes(CommandPacket& command)
{
  command.packet_type = swapBytesIfNeeded(command.packet_type);
  command.version_no = swapBytesIfNeeded(command.version_no);
  command.unused = swapBytesIfNeeded(command.unused);
  command.sequence_no = swapBytesIfNeeded(command.sequence_no);
  for (double& pos : command.command_pos)
  {
    pos = swapBytesIfNeeded(pos);
  }
  // Skip io_command since this is always expected to be little endian.
}

void swapLegacyCommandPacketBytes(LegacyCommandPacket& command)
{
  command.packet_type = swapBytesIfNeeded(command.packet_type);
  command.version_no = swapBytesIfNeeded(command.version_no);
  command.sequence_no = swapBytesIfNeeded(command.sequence_no);
  command.io_read_index = swapBytesIfNeeded(command.io_read_index);
  command.io_read_mask = swapBytesIfNeeded(command.io_read_mask);
  command.io_write_index = swapBytesIfNeeded(command.io_write_index);
  command.io_write_mask = swapBytesIfNeeded(command.io_write_mask);
  command.io_write_value = swapBytesIfNeeded(command.io_write_value);
  command.unused = swapBytesIfNeeded(command.unused);
  for (float& pos : command.command_pos)
  {
    pos = swapBytesIfNeeded(pos);
  }
}

void swapRobotStatusPacketBytes(RobotStatusPacket& status)
{
  status.packet_type = swapBytesIfNeeded(status.packet_type);
  status.version_no = swapBytesIfNeeded(status.version_no);
  status.sequence_no = swapBytesIfNeeded(status.sequence_no);
  status.time_stamp = swapBytesIfNeeded(status.time_stamp);
  status.safety_scale = swapBytesIfNeeded(status.safety_scale);
  for (int idx = 0; idx < kMaxAxisNumber; idx++)
  {
    status.joint_angle[idx] = swapBytesIfNeeded(status.joint_angle[idx]);
    status.position[idx] = swapBytesIfNeeded(status.position[idx]);
    status.current[idx] = swapBytesIfNeeded(status.current[idx]);
  }
  status.force_x = swapBytesIfNeeded(status.force_x);
  status.force_y = swapBytesIfNeeded(status.force_y);
  status.force_z = swapBytesIfNeeded(status.force_z);
  status.moment_x = swapBytesIfNeeded(status.moment_x);
  status.moment_y = swapBytesIfNeeded(status.moment_y);
  status.moment_z = swapBytesIfNeeded(status.moment_z);
  status.fs_type = swapBytesIfNeeded(status.fs_type);
  // Skip io_status since this is always expected to be little endian.
}

void swapLegacyRobotStatusPacketBytes(LegacyRobotStatusPacket& status)
{
  status.packet_type = swapBytesIfNeeded(status.packet_type);
  status.version_no = swapBytesIfNeeded(status.version_no);
  status.sequence_no = swapBytesIfNeeded(status.sequence_no);
  status.io_read_index = swapBytesIfNeeded(status.io_read_index);
  status.io_read_mask = swapBytesIfNeeded(status.io_read_mask);
  status.io_read_value = swapBytesIfNeeded(status.io_read_value);
  status.time_stamp = swapBytesIfNeeded(status.time_stamp);
  for (int idx = 0; idx < kMaxAxisNumber; idx++)
  {
    status.joint_angle[idx] = swapBytesIfNeeded(status.joint_angle[idx]);
    status.position[idx] = swapBytesIfNeeded(status.position[idx]);
    status.current[idx] = swapBytesIfNeeded(status.current[idx]);
  }
}

void swapRobotThresholdPacketBytes(RobotThresholdPacket& threshold_packet)
{
  threshold_packet.packet_type = swapBytesIfNeeded(threshold_packet.packet_type);
  threshold_packet.version_no = swapBytesIfNeeded(threshold_packet.version_no);
  threshold_packet.axis_number = swapBytesIfNeeded(threshold_packet.axis_number);
  threshold_packet.threshold_type = swapBytesIfNeeded(threshold_packet.threshold_type);
  threshold_packet.max_cartesian_speed = swapBytesIfNeeded(threshold_packet.max_cartesian_speed);
  threshold_packet.interval = swapBytesIfNeeded(threshold_packet.interval);

  for (int i = 0; i < kThresholdPayloadLength; ++i)
  {
    threshold_packet.no_payload[i] = swapBytesIfNeeded(threshold_packet.no_payload[i]);
    threshold_packet.full_payload[i] = swapBytesIfNeeded(threshold_packet.full_payload[i]);
  }
}

void swapGPIOConfigPacketBytes(GPIOConfigPacket& gpio_config_packet)
{
  gpio_config_packet.packet_type = swapBytesIfNeeded(gpio_config_packet.packet_type);
  gpio_config_packet.version_no = swapBytesIfNeeded(gpio_config_packet.version_no);
  for (auto& [command_type, gpio_type, start, length] : gpio_config_packet.gpio_configuration)
  {
    command_type = static_cast<GPIOCommandType>(swapBytesIfNeeded(static_cast<uint32_t>(command_type)));
    gpio_type = swapBytesIfNeeded(gpio_type);
    start = swapBytesIfNeeded(start);
    length = swapBytesIfNeeded(length);
  }
}

void swapControllerCapabilityBytes(ControllerCapabilityPacket& controller_capability_packet)
{
  controller_capability_packet.packet_type = swapBytesIfNeeded(controller_capability_packet.packet_type);
  controller_capability_packet.version_no = swapBytesIfNeeded(controller_capability_packet.version_no);
  controller_capability_packet.id = swapBytesIfNeeded(controller_capability_packet.id);
  controller_capability_packet.sampling_rate = swapBytesIfNeeded(controller_capability_packet.sampling_rate);
  controller_capability_packet.start_move = swapBytesIfNeeded(controller_capability_packet.start_move);
  controller_capability_packet.available_version = swapBytesIfNeeded(controller_capability_packet.available_version);
  controller_capability_packet.rob_status_use_tcp = swapBytesIfNeeded(controller_capability_packet.rob_status_use_tcp);
}

void swapControllerCapabilityResponseBytes(ControllerCapabilityResultPacket& controller_capability_result_packet)
{
  controller_capability_result_packet.packet_type = swapBytesIfNeeded(controller_capability_result_packet.packet_type);
  controller_capability_result_packet.version_no = swapBytesIfNeeded(controller_capability_result_packet.version_no);
  controller_capability_result_packet.id = swapBytesIfNeeded(controller_capability_result_packet.id);
  controller_capability_result_packet.sampling_rate =
      swapBytesIfNeeded(controller_capability_result_packet.sampling_rate);
  controller_capability_result_packet.start_move = swapBytesIfNeeded(controller_capability_result_packet.start_move);
  controller_capability_result_packet.available_version =
      swapBytesIfNeeded(controller_capability_result_packet.available_version);
  controller_capability_result_packet.rob_status_use_tcp =
      swapBytesIfNeeded(controller_capability_result_packet.rob_status_use_tcp);
}

void swapCommandPositionResponseBytes(CommandPositionResponsePacket& command_position_response)
{
  command_position_response.packet_type = swapBytesIfNeeded(command_position_response.packet_type);
  command_position_response.version_no = swapBytesIfNeeded(command_position_response.version_no);
  command_position_response.time_stamp = swapBytesIfNeeded(command_position_response.time_stamp);
  for (int idx = 0; idx < kMaxAxisNumber; ++idx)
  {
    command_position_response.position[idx] = swapBytesIfNeeded(command_position_response.position[idx]);
    command_position_response.joint_angle[idx] = swapBytesIfNeeded(command_position_response.joint_angle[idx]);
  }
}

bool StreamMotionConnection::getCommandPosition(std::array<double, kMaxAxisNumber>& command_pos) const
{
  CommandPositionRequestPacket request{};
  request.packet_type = swapBytesIfNeeded(request.packet_type);
  request.version_no = swapBytesIfNeeded(request.version_no);
  socket_impl_->send(request);

  CommandPositionResponsePacket response{};
  if (!socket_impl_->receive(response))
  {
    std::cerr << "Failed to receive command-position response." << std::endl;
    return false;
  }

  swapCommandPositionResponseBytes(response);
  if (response.packet_type != kCommandPositionPacketType)
  {
    std::cerr << "Unexpected command-position packet type: " << response.packet_type
              << " (expected " << kCommandPositionPacketType << ")." << std::endl;
    return false;
  }

  for (size_t i = 0; i < command_pos.size(); ++i)
  {
    command_pos[i] = static_cast<double>(response.joint_angle[i]);
  }
  return true;
}

void StreamMotionConnection::sendCommand(const std::array<double, kMaxAxisNumber>& command_pos,
                                         const bool is_last_command, const std::array<uint8_t, 256>& io_command) const
{
  if (command_sequence_no_ % 100 == 0 || is_last_command) {
    std::cout << "[StreamMotion] Sending CommandPacket seq=" << command_sequence_no_ << " is_last=" << is_last_command << std::endl;
  }
  if (version_no_ <= 2)
  {
    LegacyCommandPacket command{};
    command.version_no = version_no_;
    command.sequence_no = command_sequence_no_;
    command.is_last_command = is_last_command;
    command.io_read_type = 0;
    command.io_read_index = 0;
    command.io_read_mask = 0;
    command.io_write_type = 0;
    command.io_write_index = 0;
    command.io_write_mask = 0;
    command.io_write_value = 0;
    command.unused = 0;
    for (size_t i = 0; i < command.command_pos.size(); ++i)
    {
      command.command_pos[i] = static_cast<float>(command_pos[i]);
    }
    swapLegacyCommandPacketBytes(command);
    socket_impl_->send(command);
  }
  else
  {
    CommandPacket command{};
    command.version_no = version_no_;
    command.command_pos = command_pos;
    command.sequence_no = command_sequence_no_;
    command.is_last_command = is_last_command;
    command.do_motn_ctrl = 1;
    command.unused = kCommandPacketUnused;
    command.io_command = io_command;
    swapCommandPacketBytes(command);
    socket_impl_->send(command);
  }
}

bool StreamMotionConnection::getStatusPacket(RobotStatusPacket& status)
{
  static auto last_status_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_status_time).count();
  last_status_time = now;

  if (command_sequence_no_ == status_sequence_no_)
  {
    status = RobotStatusPacket{};
    bool received = false;

    // Check version_no_ and create dummy status packet if needed to keep backward compatibility
    // ROS 2 will always use the newest status packet RobotStatusPacket
    if (version_no_ <= 2)
    {
      LegacyRobotStatusPacket legacy_status{};
      received = socket_impl_->receive(legacy_status);
      if (received)
      {
        swapLegacyRobotStatusPacketBytes(legacy_status);
        status.packet_type = legacy_status.packet_type;
        status.version_no = legacy_status.version_no;
        status.sequence_no = legacy_status.sequence_no;
        status.status = legacy_status.status;
        status.robot_status = 0;
        status.contact_stop_status = ContactStopStatus::None;
        status.unused = 0;
        status.time_stamp = legacy_status.time_stamp;
        status.position = legacy_status.position;
        status.joint_angle = legacy_status.joint_angle;
        status.current = legacy_status.current;
        status.safety_scale = 0.0;
        status.force_x = 0.0;
        status.force_y = 0.0;
        status.force_z = 0.0;
        status.moment_x = 0.0;
        status.moment_y = 0.0;
        status.moment_z = 0.0;
        status.fs_type = 0;
        status.io_status.fill(0);
      }
    }
    else if (version_no_ <= 3)
    {
      V3RobotStatusPacket dummy_status{};
      received = socket_impl_->receive(dummy_status);
      if (received)
      {
        // Calculate start pointer for the last 256 bytes (io points)
        char* status_io_ptr = reinterpret_cast<char*>(&status) + (sizeof(RobotStatusPacket) - kMaxIOSize);
        char* dummy_status_io_ptr = reinterpret_cast<char*>(&dummy_status) + (sizeof(V3RobotStatusPacket) - kMaxIOSize);

        // Copy data from dummy_status to status
        std::memcpy(&status, &dummy_status, sizeof(V3RobotStatusPacket) - kMaxIOSize);
        std::memcpy(status_io_ptr, dummy_status_io_ptr, kMaxIOSize);

        // Set all the status forces to 0
        status.force_x = 0.0;
        status.force_y = 0.0;
        status.force_z = 0.0;
        status.moment_x = 0.0;
        status.moment_y = 0.0;
        status.moment_z = 0.0;
        status.fs_type = 0;
      }
    }
    else
    {
      received = socket_impl_->receive(status);
    }

    if (!received)
    {
      std::cerr << "Fail to get status packet. command_sequence_no=" << command_sequence_no_
                << " status_sequence_no=" << status_sequence_no_ << " version_no=" << version_no_ << std::endl;
      return false;
    }

    status_sequence_no_++;

    if (version_no_ > 2)
    {
      // Swap the bits of the received status packet
      swapRobotStatusPacketBytes(status);
    }

    static uint32_t last_status_bits = 0;
    static uint32_t last_robot_status_bits = 0;
    
    bool status_changed = (status.status != last_status_bits) || (status.robot_status != last_robot_status_bits);
    
    if (status_changed || status_sequence_no_ % 100 == 0) {
      bool waiting = (status.status & 0x01) != 0;
      bool cmd_received = (status.status & 0x02) != 0;
      bool sysrdy = (status.status & 0x04) != 0;
      bool moving = (status.status & 0x08) != 0;

      std::cout << "[StreamMotion] Status " << (status_changed ? "CHANGED" : "UPDATE") 
                << ": seq=" << status.sequence_no 
                << " dt=" << dt << "s"
                << " waiting=" << waiting << " cmd_received=" << cmd_received
                << " sysrdy=" << sysrdy << " moving=" << moving 
                << " status=0x" << std::hex << static_cast<int>(status.status)
                << " robot_status=0x" << static_cast<int>(status.robot_status) << std::dec << std::endl;
                
      last_status_bits = status.status;
      last_robot_status_bits = status.robot_status;
    }

    if (status_sequence_no_ != status.sequence_no)
    {
      std::cerr << "Status seq skipped. Expected seq: " << status_sequence_no_
                << " Received seq: " << status.sequence_no << std::endl;
      status_sequence_no_ = status.sequence_no;
    }
  }
  else if (command_sequence_no_ < status_sequence_no_)
  {
    std::cerr << "Command lagging behind. Command seq: " << command_sequence_no_
              << " Status seq: " << status_sequence_no_ << std::endl;
    std::cerr << "Sending extra command to catch up." << std::endl;
  }
  else
  {
    std::cerr << "Command seq exceeded status seq. Command seq: " << command_sequence_no_
              << " Status seq: " << status_sequence_no_ << std::endl;
    std::cerr << "This should not happen. Something is wrong. Need to abort." << std::endl;
    return false;
  }

  command_sequence_no_++;

  return true;
}

void ValidateGPIOConfig(const std::array<GPIOControlConfig, 32>& gpio_config)
{
  for (const auto& config : gpio_config)
  {
    if (config.command_type != GPIOCommandType::None && config.length == 0)
    {
      throw std::invalid_argument("The GPIO configuration length must be greater than 0.");
    }
  }

  for (const auto& config : gpio_config)
  {
    if ((config.command_type == GPIOCommandType::IOState || config.command_type == GPIOCommandType::IOCmd) &&
        config.gpio_type > static_cast<uint32_t>(IOType::F))
    {
      throw std::invalid_argument(
          "If the command type of the GPIO configuration is IO, the GPIO type must one of: DO, DI, RO, RI, AO, AI, F.");
    }
    if ((config.command_type == GPIOCommandType::NumRegState || config.command_type == GPIOCommandType::NumRegCmd) &&
        config.gpio_type != static_cast<uint32_t>(NumRegType::Float))
    {
      throw std::invalid_argument(
          "If the command type of the GPIO configuration is a numeric register, the GPIO type must one of: Float.");
    }
  }

  std::vector<GPIOControlConfig> read_configs;
  std::copy_if(gpio_config.begin(), gpio_config.end(), std::back_inserter(read_configs),
               [](const GPIOControlConfig& config) { return IsReadGPIOConfig(config); });
  CheckOverlapping(read_configs);

  int32_t remaining_read_bytes = 256;
  for (const auto& config : read_configs)
  {
    remaining_read_bytes -= CalculateNumBytesConfig(config);
  }
  if (remaining_read_bytes < 0)
  {
    throw std::invalid_argument("The GPIO configuration is invalid. The total amount of IO read data exceeds 256 "
                                "bytes. The provided config would take `" +
                                std::to_string(256 - remaining_read_bytes) + "` bytes.");
  }

  std::vector<GPIOControlConfig> write_configs;
  std::copy_if(gpio_config.begin(), gpio_config.end(), std::back_inserter(write_configs),
               [](const GPIOControlConfig& config) { return IsWriteGPIOConfig(config); });
  CheckOverlapping(write_configs);

  int32_t remaining_write_bytes = 256;
  for (const auto& config : write_configs)
  {
    remaining_write_bytes -= CalculateNumBytesConfig(config);
  }
  if (remaining_write_bytes < 0)
  {
    throw std::invalid_argument("The GPIO configuration is invalid. The total amount of IO write data exceeds 256 "
                                "bytes. The provided config would take `" +
                                std::to_string(256 - remaining_write_bytes) + "` bytes.");
  }
}

}  // namespace stream_motion
