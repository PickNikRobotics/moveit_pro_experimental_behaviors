// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <tl_expected/expected.hpp>

#include <cstddef>
#include <optional>
#include <string>

namespace experimental_behaviors::json_utils
{
/// Largest payload one UDP datagram can carry over IPv4.
inline constexpr std::size_t kMaxUdpPayload = 65507;

/// Owns a bound UDP socket and closes it when destroyed.
class UdpSocket
{
public:
  UdpSocket() = default;
  ~UdpSocket();
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  /// Opens a socket bound to @p address (an IPv4 or IPv6 literal, or a host name) and @p port.
  static tl::expected<UdpSocket, std::string> bind(const std::string& address, int port);

  bool isOpen() const
  {
    return fd_ >= 0;
  }

  /// Reads every datagram already queued, without waiting, and returns the last one (nullopt if none).
  tl::expected<std::optional<std::string>, std::string> receiveLatest();

private:
  explicit UdpSocket(int fd) : fd_(fd)
  {
  }
  void close();

  int fd_ = -1;
};

/// Sends @p payload to @p host and @p port as one UDP datagram.
tl::expected<void, std::string> sendDatagram(const std::string& host, int port, const std::string& payload);
}  // namespace experimental_behaviors::json_utils
