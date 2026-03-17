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
static_assert(sizeof(CommandPacketFloat) == 64);
static_assert(sizeof(CommandPacketDouble) == 104);

struct ProbeOptions
{
  std::string robot_ip;
  int timeout_ms = 1000;
  int status_count = 10;
  bool send_hold_command = true;
  bool bootstrap_rmi = true;
  std::optional<uint32_t> start_version_override;
  std::optional<std::string> command_mode_override;
};

ProbeOptions parseArgs(int argc, char** argv)
{
  if (argc < 2)
  {
    throw std::invalid_argument(
      "Usage: manual_stream_motion_probe <robot_ip> [--timeout-ms N] [--status-count N] [--no-command] "
      "[--no-rmi-bootstrap] "
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
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

void printStatusSummary(const StatusPacketV1& packet)
{
  std::cout << "Status packet: seq=" << fromBigEndian(packet.sequence_no)
            << " status=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(packet.status)
            << std::dec << " waiting=" << static_cast<int>(packet.status & 0x1 ? 1 : 0)
            << " cmd_received=" << static_cast<int>(packet.status & 0x2 ? 1 : 0)
            << " sysrdy=" << static_cast<int>(packet.status & 0x4 ? 1 : 0)
            << " moving=" << static_cast<int>(packet.status & 0x8 ? 1 : 0)
            << " timestamp_ms=" << fromBigEndian(packet.time_stamp)
            << " J1=" << fromBigEndianFloat(packet.joints[0]) << " J2=" << fromBigEndianFloat(packet.joints[1])
            << " J3=" << fromBigEndianFloat(packet.joints[2]) << std::endl;
}

void sendFloatHoldCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                          const StatusPacketV1& status, const bool last_data = false)
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
    setBigEndianFloat(command.command[i], fromBigEndianFloat(status.joints[i]));
  }

  const auto res = socket.send(&command, sizeof(command));
  if (!res || res.value() != sizeof(command))
  {
    throw std::runtime_error("Failed to send float command packet: " + res.error_message());
  }
  std::cout << "Sent float hold-position command with sequence " << sequence_no
            << " last_data=" << static_cast<int>(command.last_data) << std::endl;
}

void sendDoubleHoldCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                           const StatusPacketV1& status, const bool last_data = false)
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
    setBigEndianDouble(command.command[i], static_cast<double>(fromBigEndianFloat(status.joints[i])));
  }

  const auto res = socket.send(&command, sizeof(command));
  if (!res || res.value() != sizeof(command))
  {
    throw std::runtime_error("Failed to send double command packet: " + res.error_message());
  }
  std::cout << "Sent double hold-position command with sequence " << sequence_no
            << " last_data=" << static_cast<int>(command.last_data) << std::endl;
}

void sendHoldCommand(sockpp::udp_socket& socket, const uint32_t version, const uint32_t sequence_no,
                     const StatusPacketV1& status, const std::string& command_mode, const bool last_data = false)
{
  if (command_mode == "double")
  {
    sendDoubleHoldCommand(socket, version, sequence_no, status, last_data);
  }
  else if (command_mode == "float")
  {
    sendFloatHoldCommand(socket, version, sequence_no, status, last_data);
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

    std::cout << "Received UDP datagram: size=" << buffer.size() << " packet_type=" << packet_type
              << " version=" << version_no;
    if (buffer.size() >= 12)
    {
      std::cout << " sequence=" << sequence_no;
    }
    std::cout << " raw=" << formatBytes(buffer.data(), buffer.size()) << std::endl;

    if (buffer.size() != sizeof(StatusPacketV1))
    {
      continue;
    }

    StatusPacketV1 status{};
    std::memcpy(&status, buffer.data(), sizeof(status));
    printStatusSummary(status);
    last_status = status;

    if (options.send_hold_command && (status.status & 0x1) != 0)
    {
      const uint32_t command_sequence = fromBigEndian(status.sequence_no);
      sendHoldCommand(socket, start_version, command_sequence, status, command_mode, false);
      started_command_stream = true;
    }
  }

  if (options.send_hold_command && started_command_stream && last_status.has_value())
  {
    const uint32_t final_sequence = fromBigEndian(last_status->sequence_no);
    sendHoldCommand(socket, start_version, final_sequence, *last_status, command_mode, true);
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
