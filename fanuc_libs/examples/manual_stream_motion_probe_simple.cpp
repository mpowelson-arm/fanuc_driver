#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rmi/rmi.hpp"
#include "sockpp/inet_address.h"
#include "sockpp/socket.h"
#include "sockpp/udp_socket.h"

namespace
{
constexpr uint16_t kRobotPort = 60015;
constexpr size_t kMaxPacketBytes = 512;
constexpr int kJointIndexJ6 = 5;

bool isLittleEndian()
{
  const uint16_t test = 1;
  return *reinterpret_cast<const uint8_t*>(&test) == 1;
}

template <typename T>
T byteSwap(T value)
{
  std::array<uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  std::reverse(bytes.begin(), bytes.end());
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

template <typename T>
T toBigEndian(T value)
{
  return isLittleEndian() ? byteSwap(value) : value;
}

template <typename T>
T fromBigEndian(T value)
{
  return isLittleEndian() ? byteSwap(value) : value;
}

float fromBigEndianFloat(float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  raw = fromBigEndian(raw);
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void setBigEndianFloat(float& field, const float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  raw = toBigEndian(raw);
  std::memcpy(&field, &raw, sizeof(field));
}

std::string formatBytes(const uint8_t* data, const size_t size, const size_t max_bytes = 48)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  const size_t bytes_to_print = std::min(size, max_bytes);
  for (size_t i = 0; i < bytes_to_print; ++i)
  {
    if (i > 0)
    {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  if (size > max_bytes)
  {
    stream << " ...";
  }
  return stream.str();
}

uint32_t readHeaderField(const uint8_t* data, const size_t offset)
{
  uint32_t value = 0;
  std::memcpy(&value, data + offset, sizeof(value));
  return fromBigEndian(value);
}

#pragma pack(push, 1)

struct VersionRequestPacket
{
  uint32_t packet_type;
  uint32_t version_no;
};

struct VersionResponsePacket
{
  uint32_t packet_type;
  uint32_t version_no;
};

struct StatusStartPacket
{
  uint32_t packet_type;
  uint32_t version_no;
};

struct StatusStopPacket
{
  uint32_t packet_type;
  uint32_t version_no;
};

struct CommandPositionRequestPacket
{
  uint32_t packet_type;
  uint32_t version_no;
};

struct CommandPositionResponsePacket
{
  uint32_t packet_type;
  uint32_t version_no;
  uint32_t time_stamp;
  std::array<float, 9> cart{};
  std::array<float, 9> joints{};
};

struct StatusPacket
{
  uint32_t packet_type;
  uint32_t version_no;
  uint32_t sequence_no;
  uint8_t status;
  uint8_t io_read_type;
  uint16_t io_read_index;
  uint16_t io_read_mask;
  uint16_t io_read_value;
  uint32_t time_stamp;
  std::array<float, 9> cart{};
  std::array<float, 9> joints{};
  std::array<float, 9> currents{};
};

struct CommandPacketFloat
{
  uint32_t packet_type;
  uint32_t version_no;
  uint32_t sequence_no;
  uint8_t last_data;
  uint8_t io_read_type;
  uint16_t io_read_index;
  uint16_t io_read_mask;
  uint8_t data_format;
  uint8_t io_write_type;
  uint16_t io_write_index;
  uint16_t io_write_mask;
  uint16_t io_write_value;
  uint16_t unused;
  std::array<float, 9> command{};
};

#pragma pack(pop)

static_assert(sizeof(VersionRequestPacket) == 8);
static_assert(sizeof(CommandPositionRequestPacket) == 8);
static_assert(sizeof(CommandPositionResponsePacket) == 84);
static_assert(sizeof(StatusPacket) == 132);
static_assert(sizeof(CommandPacketFloat) == 64);

struct SimpleProbeOptions
{
  std::string robot_ip;
  int timeout_ms = 200;
  int max_status_count = 20000;
  int log_every = 50;
  bool bootstrap_rmi = true;
  double j6_delta_deg = -10.0;
  int ramp_duration_ms = 2000;
  int settle_timeout_ms = 2000;
};

SimpleProbeOptions parseArgs(int argc, char** argv)
{
  if (argc < 2)
  {
    throw std::invalid_argument(
      "Usage: manual_stream_motion_probe_simple <robot_ip> [--timeout-ms N] [--max-status-count N] [--log-every N] "
      "[--no-rmi-bootstrap] [--j6-delta-deg N] [--ramp-duration-ms N] [--settle-timeout-ms N]");
  }

  SimpleProbeOptions options;
  options.robot_ip = argv[1];
  for (int i = 2; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--timeout-ms" && i + 1 < argc)
    {
      options.timeout_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--max-status-count" && i + 1 < argc)
    {
      options.max_status_count = std::stoi(argv[++i]);
    }
    else if (arg == "--log-every" && i + 1 < argc)
    {
      options.log_every = std::stoi(argv[++i]);
    }
    else if (arg == "--j6-delta-deg" && i + 1 < argc)
    {
      options.j6_delta_deg = std::stod(argv[++i]);
    }
    else if (arg == "--ramp-duration-ms" && i + 1 < argc)
    {
      options.ramp_duration_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--settle-timeout-ms" && i + 1 < argc)
    {
      options.settle_timeout_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--no-rmi-bootstrap")
    {
      options.bootstrap_rmi = false;
    }
    else
    {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }

  return options;
}

void bootstrapStreamMotionViaRMI(const SimpleProbeOptions& options)
{
  std::cout << "Bootstrapping Stream Motion via RMI." << std::endl;
  rmi::RMIConnection rmi_connection(options.robot_ip);
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
}

void dumpRMIState(const SimpleProbeOptions& options, const std::string& reason)
{
  try
  {
    rmi::RMIConnection rmi_connection(options.robot_ip);
    rmi_connection.connect(5.0);
    const auto status = rmi_connection.getStatus(5.0);
    std::cout << "RMI status after " << reason << ":"
              << " ErrorID=" << status.ErrorID
              << " ServoReady=" << static_cast<int>(status.ServoReady)
              << " TPMode=" << static_cast<int>(status.TPMode)
              << " RMIMotionStatus=" << static_cast<int>(status.RMIMotionStatus)
              << " ProgramStatus=" << static_cast<int>(status.ProgramStatus)
              << " Override=" << static_cast<int>(status.Override)
              << std::endl;

    const auto error = rmi_connection.readError(5.0);
    std::cout << "RMI readError after " << reason << ":"
              << " ErrorID=" << error.ErrorID
              << " ErrorData=" << error.ErrorData;
    if (error.ErrorData2.has_value())
    {
      std::cout << " ErrorData2=" << *error.ErrorData2;
    }
    std::cout << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cout << "Failed to query RMI state after " << reason << ": " << e.what() << std::endl;
  }
}

bool receivePacket(sockpp::udp_socket& socket, std::vector<uint8_t>& buffer, const int timeout_ms)
{
  buffer.assign(kMaxPacketBytes, 0);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms))
  {
    const auto res = socket.recv(buffer.data(), buffer.size());
    if (res && res.value() > 0)
    {
      buffer.resize(res.value());
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return false;
}

std::optional<CommandPositionResponsePacket> requestCommandPosition(sockpp::udp_socket& socket, const int timeout_ms)
{
  CommandPositionRequestPacket request{ toBigEndian<uint32_t>(4), toBigEndian<uint32_t>(1) };
  const auto send_res = socket.send(&request, sizeof(request));
  if (!send_res || send_res.value() != sizeof(request))
  {
    throw std::runtime_error("Failed to send command-position request packet: " + send_res.error_message());
  }

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms))
  {
    std::vector<uint8_t> buffer;
    if (!receivePacket(socket, buffer, 100))
    {
      continue;
    }

    const uint32_t packet_type = buffer.size() >= 4 ? readHeaderField(buffer.data(), 0) : 0xFFFFFFFF;
    if (packet_type != 4 || buffer.size() != sizeof(CommandPositionResponsePacket))
    {
      continue;
    }

    CommandPositionResponsePacket response{};
    std::memcpy(&response, buffer.data(), sizeof(response));
    return response;
  }

  return std::nullopt;
}

void logStatus(const StatusPacket& status, const int packet_index)
{
  std::cout << "Status packet[" << packet_index << "]: seq=" << fromBigEndian(status.sequence_no)
            << " status=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(status.status)
            << std::dec
            << " waiting=" << static_cast<int>((status.status & 0x1) != 0)
            << " cmd_received=" << static_cast<int>((status.status & 0x2) != 0)
            << " sysrdy=" << static_cast<int>((status.status & 0x4) != 0)
            << " moving=" << static_cast<int>((status.status & 0x8) != 0)
            << " timestamp_ms=" << fromBigEndian(status.time_stamp)
            << " J6=" << fromBigEndianFloat(status.joints[kJointIndexJ6])
            << " J7=" << fromBigEndianFloat(status.joints[6])
            << std::endl;
}

void sendFloatCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                      const std::array<float, 9>& joints, const bool last_data, const int packet_index)
{
  CommandPacketFloat command{};
  command.packet_type = toBigEndian<uint32_t>(1);
  command.version_no = toBigEndian(version);
  command.sequence_no = toBigEndian(sequence_no);
  command.last_data = last_data ? 1 : 0;
  command.io_read_type = 0;
  command.io_read_index = toBigEndian<uint16_t>(0);
  command.io_read_mask = toBigEndian<uint16_t>(0);
  command.data_format = 1;
  command.io_write_type = 0;
  command.io_write_index = toBigEndian<uint16_t>(0);
  command.io_write_mask = toBigEndian<uint16_t>(0);
  command.io_write_value = toBigEndian<uint16_t>(0);
  command.unused = toBigEndian<uint16_t>(0xFFFF);
  for (size_t i = 0; i < joints.size(); ++i)
  {
    setBigEndianFloat(command.command[i], joints[i]);
  }

  const auto send_res = socket.send(&command, sizeof(command));
  if (!send_res || send_res.value() != sizeof(command))
  {
    throw std::runtime_error("Failed to send float command packet: " + send_res.error_message());
  }

  std::cout << "Sent float command[" << packet_index << "] seq=" << sequence_no
            << " last_data=" << static_cast<int>(command.last_data)
            << " J6=" << joints[kJointIndexJ6]
            << std::endl;
}

std::array<float, 9> getRampTargetJoints(const std::array<float, 9>& seed_joints, const StatusPacket& status,
                                         const SimpleProbeOptions& options, const uint32_t initial_timestamp_ms)
{
  std::array<float, 9> target_joints = seed_joints;
  const uint32_t current_timestamp_ms = fromBigEndian(status.time_stamp);
  const double elapsed_ms = static_cast<double>(current_timestamp_ms - initial_timestamp_ms);
  const double progress =
    std::clamp(elapsed_ms / static_cast<double>(std::max(1, options.ramp_duration_ms)), 0.0, 1.0);
  target_joints[kJointIndexJ6] =
    static_cast<float>(seed_joints[kJointIndexJ6] + options.j6_delta_deg * progress);
  return target_joints;
}

}  // namespace

int main(int argc, char** argv)
{
  try
  {
    const SimpleProbeOptions options = parseArgs(argc, argv);
    sockpp::initialize();

    if (options.bootstrap_rmi)
    {
      bootstrapStreamMotionViaRMI(options);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    sockpp::udp_socket socket;
    const sockpp::inet_address robot_address(options.robot_ip, kRobotPort);
    if (const auto res = socket.connect(robot_address); !res)
    {
      throw std::runtime_error("Failed to connect UDP socket: " + res.error_message());
    }
    socket.set_non_blocking(true);
    std::cout << "Connected UDP socket. local=" << socket.address() << " remote=" << robot_address << std::endl;

    VersionRequestPacket version_request{ toBigEndian<uint32_t>(6), toBigEndian<uint32_t>(1) };
    const auto version_send_res = socket.send(&version_request, sizeof(version_request));
    if (!version_send_res || version_send_res.value() != sizeof(version_request))
    {
      throw std::runtime_error("Failed to send version request packet: " + version_send_res.error_message());
    }
    std::cout << "Sent version request packet." << std::endl;

    uint32_t session_version = 2;
    std::vector<uint8_t> version_response_bytes;
    if (receivePacket(socket, version_response_bytes, options.timeout_ms))
    {
      const uint32_t packet_type =
        version_response_bytes.size() >= 4 ? readHeaderField(version_response_bytes.data(), 0) : 0xFFFFFFFF;
      const uint32_t version_no =
        version_response_bytes.size() >= 8 ? readHeaderField(version_response_bytes.data(), 4) : 0xFFFFFFFF;
      std::cout << "Version request response: size=" << version_response_bytes.size()
                << " packet_type=" << packet_type
                << " version=" << version_no
                << " raw=" << formatBytes(version_response_bytes.data(), version_response_bytes.size())
                << std::endl;
      if (packet_type == 6 && version_response_bytes.size() == sizeof(VersionResponsePacket) && version_no == 1)
      {
        session_version = 1;
      }
    }
    else
    {
      std::cout << "No response to version request within " << options.timeout_ms << " ms. Defaulting to version 2."
                << std::endl;
    }

    StatusStartPacket start_packet{ toBigEndian<uint32_t>(0), toBigEndian(session_version) };
    const auto start_send_res = socket.send(&start_packet, sizeof(start_packet));
    if (!start_send_res || start_send_res.value() != sizeof(start_packet))
    {
      throw std::runtime_error("Failed to send status-start packet: " + start_send_res.error_message());
    }
    std::cout << "Sent status-start packet with version " << session_version << std::endl;

    std::optional<std::array<float, 9>> seed_joints;
    std::optional<uint32_t> initial_timestamp_ms;
    if (const auto command_position = requestCommandPosition(socket, options.timeout_ms))
    {
      std::array<float, 9> joints{};
      for (size_t i = 0; i < joints.size(); ++i)
      {
        joints[i] = fromBigEndianFloat(command_position->joints[i]);
      }
      seed_joints = joints;
      std::cout << "Seeded commands from command-position response."
                << " J6=" << joints[kJointIndexJ6]
                << " J7=" << joints[6]
                << std::endl;
    }

    bool started_command_stream = false;
    uint32_t next_command_sequence = 0;

    bool sent_last_data = false;
    int last_data_packet_index = -1;
    std::optional<std::chrono::steady_clock::time_point> last_data_sent_time;

    for (int i = 0; i < options.max_status_count; ++i)
    {
      std::vector<uint8_t> buffer;
      if (!receivePacket(socket, buffer, options.timeout_ms))
      {
        std::cout << "No UDP packet received within " << options.timeout_ms << " ms." << std::endl;
        break;
      }

      if (i == 0 || (options.log_every > 0 && i % options.log_every == 0))
      {
        const uint32_t packet_type = buffer.size() >= 4 ? readHeaderField(buffer.data(), 0) : 0xFFFFFFFF;
        const uint32_t version_no = buffer.size() >= 8 ? readHeaderField(buffer.data(), 4) : 0xFFFFFFFF;
        const uint32_t sequence_no = buffer.size() >= 12 ? readHeaderField(buffer.data(), 8) : 0xFFFFFFFF;
        std::cout << "Received UDP datagram[" << i << "]: size=" << buffer.size()
                  << " packet_type=" << packet_type
                  << " version=" << version_no
                  << " sequence=" << sequence_no
                  << " raw=" << formatBytes(buffer.data(), buffer.size())
                  << std::endl;
      }

      if (buffer.size() != sizeof(StatusPacket))
      {
        continue;
      }

      StatusPacket status{};
      std::memcpy(&status, buffer.data(), sizeof(status));
      if (i == 0 || (options.log_every > 0 && i % options.log_every == 0))
      {
        logStatus(status, i);
      }

      if (!seed_joints.has_value())
      {
        std::array<float, 9> joints{};
        for (size_t joint_idx = 0; joint_idx < joints.size(); ++joint_idx)
        {
          joints[joint_idx] = fromBigEndianFloat(status.joints[joint_idx]);
        }
        seed_joints = joints;
        std::cout << "Falling back to status joints for seed."
                  << " J6=" << joints[kJointIndexJ6]
                  << " J7=" << joints[6]
                  << std::endl;
      }
      if (!initial_timestamp_ms.has_value())
      {
        initial_timestamp_ms = fromBigEndian(status.time_stamp);
      }

      const bool waiting = (status.status & 0x1) != 0;
      if (waiting)
      {
        if (!started_command_stream)
        {
          next_command_sequence = fromBigEndian(status.sequence_no);
          started_command_stream = true;
        }

        const uint32_t current_timestamp_ms = fromBigEndian(status.time_stamp);
        const uint32_t elapsed_ms = current_timestamp_ms - *initial_timestamp_ms;
        const auto target_joints = getRampTargetJoints(*seed_joints, status, options, *initial_timestamp_ms);
        const bool ramp_complete = elapsed_ms >= static_cast<uint32_t>(std::max(0, options.ramp_duration_ms));

        if (!ramp_complete)
        {
          sendFloatCommand(socket, session_version, next_command_sequence, target_joints, false, i);
          ++next_command_sequence;
        }
        else if (!sent_last_data)
        {
          sendFloatCommand(socket, session_version, next_command_sequence, target_joints, true, i);
          ++next_command_sequence;
          sent_last_data = true;
          last_data_packet_index = i;
          last_data_sent_time = std::chrono::steady_clock::now();
          std::cout << "Ramp complete. Sent final command with last_data=1 at status packet[" << i << "]."
                    << std::endl;
        }
        else if (last_data_sent_time.has_value() &&
                 std::chrono::steady_clock::now() - *last_data_sent_time >
                 std::chrono::milliseconds(std::max(1, options.settle_timeout_ms)))
        {
          std::cout << "Timed out waiting for stream to end after last_data=1." << std::endl;
          dumpRMIState(options, "settle timeout");
          break;
        }
      }
      else if (started_command_stream)
      {
        if (sent_last_data)
        {
          std::cout << "Waiting bit dropped at status packet[" << i
                    << "] after final command at packet[" << last_data_packet_index << "]." << std::endl;
        }
        else
        {
          std::cout << "Waiting bit dropped at status packet[" << i << "]." << std::endl;
        }
        dumpRMIState(options, "waiting bit drop");
        break;
      }
    }

    StatusStopPacket stop_packet{ toBigEndian<uint32_t>(2), toBigEndian(session_version) };
    const auto stop_send_res = socket.send(&stop_packet, sizeof(stop_packet));
    if (!stop_send_res || stop_send_res.value() != sizeof(stop_packet))
    {
      throw std::runtime_error("Failed to send status-stop packet: " + stop_send_res.error_message());
    }
    std::cout << "Sent status-stop packet with version " << session_version << std::endl;
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
