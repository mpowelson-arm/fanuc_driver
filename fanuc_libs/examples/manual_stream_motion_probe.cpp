#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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

double fromBigEndianDouble(double value)
{
  uint64_t raw = 0;
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

void setBigEndianDouble(double& field, const double value)
{
  uint64_t raw = 0;
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

struct StatusPacketV1
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

struct CommandPacketDouble
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
  uint16_t unused16;
  uint32_t unused32;
  std::array<double, 9> command{};
};

#pragma pack(pop)

static_assert(sizeof(VersionRequestPacket) == 8);
static_assert(sizeof(StatusPacketV1) == 132);
static_assert(sizeof(CommandPositionRequestPacket) == 8);
static_assert(sizeof(CommandPositionResponsePacket) == 84);
static_assert(sizeof(CommandPacketFloat) == 64);
static_assert(sizeof(CommandPacketDouble) == 104);

struct ProbeOptions
{
  std::string robot_ip;
  int timeout_ms = 1000;
  int status_count = 10;
  bool send_hold_command = true;
  bool bootstrap_rmi = true;
  bool wiggle_j6 = false;
  double j6_amplitude_deg = 10.0;
  int sine_period_ms = 2000;
  int joint_number = 6;
  std::optional<double> joint_delta_deg;
  int rotate_duration_ms = 5000;
  int start_delay_ms = 500;
  int log_every = 25;
  std::optional<uint32_t> start_version_override;
  std::optional<std::string> command_mode_override;
};

ProbeOptions parseArgs(int argc, char** argv)
{
  if (argc < 2)
  {
    throw std::invalid_argument(
      "Usage: manual_stream_motion_probe <robot_ip> [--timeout-ms N] [--status-count N] [--no-command] "
      "[--no-rmi-bootstrap] [--wiggle-j6] [--j6-amplitude-deg N] [--sine-period-ms N] "
      "[--joint-number N] [--joint-delta-deg N] [--rotate-duration-ms N] [--start-delay-ms N] [--log-every N] "
      "[--start-version N] [--command-mode float|double]");
  }

  ProbeOptions options;
  options.robot_ip = argv[1];
  for (int i = 2; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--timeout-ms" && i + 1 < argc)
    {
      options.timeout_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--status-count" && i + 1 < argc)
    {
      options.status_count = std::stoi(argv[++i]);
    }
    else if (arg == "--no-command")
    {
      options.send_hold_command = false;
    }
    else if (arg == "--no-rmi-bootstrap")
    {
      options.bootstrap_rmi = false;
    }
    else if (arg == "--wiggle-j6")
    {
      options.wiggle_j6 = true;
    }
    else if (arg == "--j6-amplitude-deg" && i + 1 < argc)
    {
      options.j6_amplitude_deg = std::stod(argv[++i]);
    }
    else if (arg == "--sine-period-ms" && i + 1 < argc)
    {
      options.sine_period_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--joint-number" && i + 1 < argc)
    {
      options.joint_number = std::stoi(argv[++i]);
    }
    else if (arg == "--joint-delta-deg" && i + 1 < argc)
    {
      options.joint_delta_deg = std::stod(argv[++i]);
    }
    else if (arg == "--rotate-duration-ms" && i + 1 < argc)
    {
      options.rotate_duration_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--start-delay-ms" && i + 1 < argc)
    {
      options.start_delay_ms = std::stoi(argv[++i]);
    }
    else if (arg == "--log-every" && i + 1 < argc)
    {
      options.log_every = std::stoi(argv[++i]);
    }
    else if (arg == "--start-version" && i + 1 < argc)
    {
      options.start_version_override = static_cast<uint32_t>(std::stoul(argv[++i]));
    }
    else if (arg == "--command-mode" && i + 1 < argc)
    {
      options.command_mode_override = argv[++i];
    }
    else
    {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }

  return options;
}

void bootstrapStreamMotionViaRMI(const ProbeOptions& options)
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

bool receivePacket(sockpp::udp_socket& socket, std::vector<uint8_t>& buffer, const int timeout_ms)
{
  buffer.assign(kMaxPacketBytes, 0);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms))
  {
    auto res = socket.recv(buffer.data(), buffer.size());
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

void printStatusSummary(const StatusPacketV1& packet, const int packet_index)
{
  std::cout << "Status packet[" << packet_index << "]: seq=" << fromBigEndian(packet.sequence_no)
            << " status=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(packet.status)
            << std::dec << " waiting=" << static_cast<int>(packet.status & 0x1 ? 1 : 0)
            << " cmd_received=" << static_cast<int>(packet.status & 0x2 ? 1 : 0)
            << " sysrdy=" << static_cast<int>(packet.status & 0x4 ? 1 : 0)
            << " moving=" << static_cast<int>(packet.status & 0x8 ? 1 : 0)
            << " timestamp_ms=" << fromBigEndian(packet.time_stamp);
  for (size_t i = 0; i < 6; ++i)
  {
    std::cout << " J" << (i + 1) << '=' << fromBigEndianFloat(packet.joints[i]);
  }
  std::cout << std::endl;
}

std::array<float, 9> getCommandJoints(const StatusPacketV1& status, const ProbeOptions& options,
                                      const std::optional<std::array<float, 9>>& initial_joints,
                                      const uint32_t initial_timestamp_ms)
{
  constexpr double kPi = 3.14159265358979323846;
  std::array<float, 9> command_joints{};
  for (size_t i = 0; i < command_joints.size(); ++i)
  {
    command_joints[i] = initial_joints.has_value() ? (*initial_joints)[i] : fromBigEndianFloat(status.joints[i]);
  }

  const size_t joint_index = static_cast<size_t>(options.joint_number - 1);
  if (joint_index >= command_joints.size())
  {
    throw std::out_of_range("joint-number must be between 1 and 9.");
  }

  if (options.joint_delta_deg.has_value())
  {
    const uint32_t current_timestamp_ms = fromBigEndian(status.time_stamp);
    const double elapsed_ms = static_cast<double>(current_timestamp_ms - initial_timestamp_ms);
    const double move_elapsed_ms = std::max(0.0, elapsed_ms - static_cast<double>(options.start_delay_ms));
    const double u = std::clamp(move_elapsed_ms / static_cast<double>(options.rotate_duration_ms), 0.0, 1.0);
    const double progress = (10.0 * u * u * u) - (15.0 * u * u * u * u) + (6.0 * u * u * u * u * u);
    command_joints[joint_index] =
      static_cast<float>(command_joints[joint_index] + (*options.joint_delta_deg * progress));
  }
  else if (options.wiggle_j6)
  {
    const uint32_t current_timestamp_ms = fromBigEndian(status.time_stamp);
    const double time_seconds = static_cast<double>(current_timestamp_ms - initial_timestamp_ms) / 1000.0;
    const double omega = (2.0 * kPi) / (static_cast<double>(options.sine_period_ms) / 1000.0);
    command_joints[5] = static_cast<float>(command_joints[5] + options.j6_amplitude_deg * std::sin(omega * time_seconds));
  }

  return command_joints;
}

void sendFloatCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                      const std::array<float, 9>& command_joints, const int joint_number, const int packet_index,
                      const int log_every, const bool last_data = false)
{
  CommandPacketFloat command{};
  command.packet_type = toBigEndian<uint32_t>(1);
  command.version_no = toBigEndian(version);
  command.sequence_no = toBigEndian(sequence_no);
  command.last_data = last_data ? 1 : 0;
  command.io_read_type = 0;
  command.io_read_index = toBigEndian<uint16_t>(0);
  command.io_read_mask = toBigEndian<uint16_t>(0);
  command.data_format = 1;  // Joint
  command.io_write_type = 0;
  command.io_write_index = toBigEndian<uint16_t>(0);
  command.io_write_mask = toBigEndian<uint16_t>(0);
  command.io_write_value = toBigEndian<uint16_t>(0);
  command.unused = toBigEndian<uint16_t>(0);
  for (size_t i = 0; i < command.command.size(); ++i)
  {
    setBigEndianFloat(command.command[i], command_joints[i]);
  }

  const auto res = socket.send(&command, sizeof(command));
  if (!res || res.value() != sizeof(command))
  {
    throw std::runtime_error("Failed to send float command packet: " + res.error_message());
  }
  if (last_data || packet_index == 0 || (log_every > 0 && packet_index % log_every == 0))
  {
    std::cout << "Sent float command[" << packet_index << "] with sequence " << sequence_no
              << " last_data=" << static_cast<int>(command.last_data) << " J" << joint_number << "="
              << command_joints[static_cast<size_t>(joint_number - 1)] << std::endl;
  }
}

void sendDoubleCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                       const std::array<float, 9>& command_joints, const int joint_number, const int packet_index,
                       const int log_every, const bool last_data = false)
{
  CommandPacketDouble command{};
  command.packet_type = toBigEndian<uint32_t>(5);
  command.version_no = toBigEndian(version);
  command.sequence_no = toBigEndian(sequence_no);
  command.last_data = last_data ? 1 : 0;
  command.io_read_type = 0;
  command.io_read_index = toBigEndian<uint16_t>(0);
  command.io_read_mask = toBigEndian<uint16_t>(0);
  command.data_format = 1;  // Joint only for v2+
  command.io_write_type = 0;
  command.io_write_index = toBigEndian<uint16_t>(0);
  command.io_write_mask = toBigEndian<uint16_t>(0);
  command.io_write_value = toBigEndian<uint16_t>(0);
  command.unused16 = toBigEndian<uint16_t>(0);
  command.unused32 = toBigEndian<uint32_t>(0);
  for (size_t i = 0; i < command.command.size(); ++i)
  {
    setBigEndianDouble(command.command[i], static_cast<double>(command_joints[i]));
  }

  const auto res = socket.send(&command, sizeof(command));
  if (!res || res.value() != sizeof(command))
  {
    throw std::runtime_error("Failed to send double command packet: " + res.error_message());
  }
  if (last_data || packet_index == 0 || (log_every > 0 && packet_index % log_every == 0))
  {
    std::cout << "Sent double command[" << packet_index << "] with sequence " << sequence_no
              << " last_data=" << static_cast<int>(command.last_data) << " J" << joint_number << "="
              << command_joints[static_cast<size_t>(joint_number - 1)] << std::endl;
  }
}

void sendCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                 const std::array<float, 9>& command_joints, const int joint_number, const int packet_index,
                 const int log_every, const std::string& command_mode, const bool last_data = false)
{
  if (command_mode == "double")
  {
    sendDoubleCommand(socket, version, sequence_no, command_joints, joint_number, packet_index, log_every, last_data);
  }
  else if (command_mode == "float")
  {
    sendFloatCommand(socket, version, sequence_no, command_joints, joint_number, packet_index, log_every, last_data);
  }
  else
  {
    throw std::invalid_argument("Unsupported command mode: " + command_mode);
  }
}

bool runStatusProbe(sockpp::udp_socket& socket, const ProbeOptions& options, const uint32_t start_version)
{
  StatusStartPacket start_packet{ toBigEndian<uint32_t>(0), toBigEndian(start_version) };
  auto send_res = socket.send(&start_packet, sizeof(start_packet));
  if (!send_res || send_res.value() != sizeof(start_packet))
  {
    throw std::runtime_error("Failed to send status-start packet: " + send_res.error_message());
  }

  std::cout << "Sent status-start packet with version " << start_version << std::endl;

  bool started_command_stream = false;
  bool got_status = false;
  std::string command_mode = options.command_mode_override.value_or(start_version >= 2 ? "double" : "float");
  std::optional<StatusPacketV1> last_status;
  std::optional<std::array<float, 9>> initial_joints;
  std::optional<uint32_t> initial_timestamp_ms;
  std::optional<uint32_t> next_command_sequence;

  if (const auto command_position = requestCommandPosition(socket, options.timeout_ms))
  {
    std::array<float, 9> seed{};
    for (size_t joint_idx = 0; joint_idx < seed.size(); ++joint_idx)
    {
      seed[joint_idx] = fromBigEndianFloat(command_position->joints[joint_idx]);
    }
    initial_joints = seed;
    std::cout << "Seeded commands from command-position response."
              << " J1=" << seed[0] << " J2=" << seed[1] << " J3=" << seed[2] << " J4=" << seed[3]
              << " J5=" << seed[4] << " J6=" << seed[5] << " J7=" << seed[6] << std::endl;
  }
  else
  {
    std::cout << "Command-position response not received. Will fall back to status/servo position for initial command seed."
              << std::endl;
  }

  for (int i = 0; i < options.status_count; ++i)
  {
    std::vector<uint8_t> buffer;
    if (!receivePacket(socket, buffer, options.timeout_ms))
    {
      std::cout << "No UDP packet received within " << options.timeout_ms << " ms after start version " << start_version
                << std::endl;
      break;
    }

    got_status = true;
    const uint32_t packet_type = buffer.size() >= 4 ? readHeaderField(buffer.data(), 0) : 0xFFFFFFFF;
    const uint32_t version_no = buffer.size() >= 8 ? readHeaderField(buffer.data(), 4) : 0xFFFFFFFF;
    const uint32_t sequence_no = buffer.size() >= 12 ? readHeaderField(buffer.data(), 8) : 0xFFFFFFFF;

    if (i == 0 || (options.log_every > 0 && i % options.log_every == 0))
    {
      std::cout << "Received UDP datagram[" << i << "]: size=" << buffer.size() << " packet_type=" << packet_type
                << " version=" << version_no;
      if (buffer.size() >= 12)
      {
        std::cout << " sequence=" << sequence_no;
      }
      std::cout << " raw=" << formatBytes(buffer.data(), buffer.size()) << std::endl;
    }

    if (buffer.size() != sizeof(StatusPacketV1))
    {
      continue;
    }

    StatusPacketV1 status{};
    std::memcpy(&status, buffer.data(), sizeof(status));
    if (i == 0 || (options.log_every > 0 && i % options.log_every == 0))
    {
      printStatusSummary(status, i);
    }
    last_status = status;
    if (!initial_joints.has_value())
    {
      std::array<float, 9> seed{};
      for (size_t joint_idx = 0; joint_idx < seed.size(); ++joint_idx)
      {
        seed[joint_idx] = fromBigEndianFloat(status.joints[joint_idx]);
      }
      initial_joints = seed;
      std::cout << "Falling back to status/servo position for initial command seed."
                << " J1=" << seed[0] << " J2=" << seed[1] << " J3=" << seed[2] << " J4=" << seed[3]
                << " J5=" << seed[4] << " J6=" << seed[5] << " J7=" << seed[6] << std::endl;
      initial_timestamp_ms = fromBigEndian(status.time_stamp);
    }
    if (!initial_timestamp_ms.has_value())
    {
      initial_timestamp_ms = fromBigEndian(status.time_stamp);
    }

    if (options.send_hold_command && (status.status & 0x1) != 0 && initial_timestamp_ms.has_value())
    {
      if (!next_command_sequence.has_value())
      {
        next_command_sequence = fromBigEndian(status.sequence_no);
      }
      const auto command_joints = getCommandJoints(status, options, initial_joints, *initial_timestamp_ms);
      sendCommand(socket, start_version, *next_command_sequence, command_joints, options.joint_number, i,
                  options.log_every, command_mode, false);
      *next_command_sequence = *next_command_sequence + 1;
      started_command_stream = true;
    }
  }

  if (options.send_hold_command && started_command_stream && last_status.has_value() && initial_joints.has_value() &&
      initial_timestamp_ms.has_value() && next_command_sequence.has_value())
  {
    const auto command_joints = getCommandJoints(*last_status, options, initial_joints, *initial_timestamp_ms);
    sendCommand(socket, start_version, *next_command_sequence, command_joints, options.joint_number, options.status_count,
                options.log_every, command_mode, true);
  }

  StatusStopPacket stop_packet{ toBigEndian<uint32_t>(2), toBigEndian(start_version) };
  send_res = socket.send(&stop_packet, sizeof(stop_packet));
  if (!send_res || send_res.value() != sizeof(stop_packet))
  {
    throw std::runtime_error("Failed to send status-stop packet: " + send_res.error_message());
  }
  std::cout << "Sent status-stop packet with version " << start_version << std::endl;

  return got_status;
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    const ProbeOptions options = parseArgs(argc, argv);
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
    auto send_res = socket.send(&version_request, sizeof(version_request));
    if (!send_res || send_res.value() != sizeof(version_request))
    {
      throw std::runtime_error("Failed to send version request packet: " + send_res.error_message());
    }
    std::cout << "Sent version request packet." << std::endl;

    std::optional<uint32_t> available_version;
    std::vector<uint8_t> response;
    if (receivePacket(socket, response, options.timeout_ms))
    {
      const uint32_t packet_type = response.size() >= 4 ? readHeaderField(response.data(), 0) : 0xFFFFFFFF;
      const uint32_t version_no = response.size() >= 8 ? readHeaderField(response.data(), 4) : 0xFFFFFFFF;
      std::cout << "Version request response: size=" << response.size() << " packet_type=" << packet_type
                << " version=" << version_no << " raw=" << formatBytes(response.data(), response.size()) << std::endl;
      if (packet_type == 6 && response.size() == sizeof(VersionResponsePacket))
      {
        available_version = version_no;
      }
    }
    else
    {
      std::cout << "No response to version request within " << options.timeout_ms << " ms." << std::endl;
    }

    std::vector<uint32_t> versions_to_try;
    if (options.start_version_override.has_value())
    {
      versions_to_try.push_back(*options.start_version_override);
    }
    else if (available_version.has_value() && *available_version > 1)
    {
      versions_to_try.push_back(*available_version);
      versions_to_try.push_back(1);
    }
    else
    {
      versions_to_try.push_back(1);
    }

    for (const uint32_t version : versions_to_try)
    {
      if (runStatusProbe(socket, options, version))
      {
        return 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cerr << "Did not receive any status packets for start versions:";
    for (const uint32_t version : versions_to_try)
    {
      std::cerr << ' ' << version;
    }
    std::cerr << std::endl;
    return 2;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
