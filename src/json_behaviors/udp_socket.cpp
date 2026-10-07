// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <experimental_behaviors/json_behaviors/udp_socket.hpp>

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <memory>

namespace experimental_behaviors::json_utils
{
namespace
{
using AddrInfoPtr = std::unique_ptr<addrinfo, decltype(&freeaddrinfo)>;

tl::expected<AddrInfoPtr, std::string> resolve(const std::string& host, int port, bool passive)
{
  if (port < 1 || port > 65535)
  {
    return tl::make_unexpected("port " + std::to_string(port) + " is outside 1-65535");
  }
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_NUMERICSERV | (passive ? AI_PASSIVE : 0);
  addrinfo* result = nullptr;
  const int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
  if (rc != 0)
  {
    return tl::make_unexpected("cannot resolve '" + host + "': " + gai_strerror(rc));
  }
  return AddrInfoPtr(result, &freeaddrinfo);
}

std::string errnoMessage(const std::string& what)
{
  return what + ": " + std::strerror(errno);
}
}  // namespace

UdpSocket::~UdpSocket()
{
  close();
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept : fd_(other.fd_)
{
  other.fd_ = -1;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
{
  if (this != &other)
  {
    close();
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void UdpSocket::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

tl::expected<UdpSocket, std::string> UdpSocket::bind(const std::string& address, int port)
{
  const auto addresses = resolve(address, port, true);
  if (!addresses)
  {
    return tl::make_unexpected(addresses.error());
  }
  std::string last_error = "no address to bind";
  for (const addrinfo* ai = addresses->get(); ai != nullptr; ai = ai->ai_next)
  {
    UdpSocket socket(::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol));
    if (!socket.isOpen())
    {
      last_error = errnoMessage("socket()");
      continue;
    }
    // No SO_REUSEADDR: on Linux it would let a second socket bind this UDP port and take its datagrams.
    if (::bind(socket.fd_, ai->ai_addr, ai->ai_addrlen) != 0)
    {
      last_error = errnoMessage("bind()");
      continue;
    }
    return socket;
  }
  return tl::make_unexpected("cannot bind UDP " + address + ":" + std::to_string(port) + ": " + last_error);
}

tl::expected<std::optional<std::string>, std::string> UdpSocket::receiveLatest()
{
  if (!isOpen())
  {
    return tl::make_unexpected("the socket is not open");
  }
  // Large enough for any UDP payload, so no datagram is truncated.
  static thread_local std::array<char, 65536> buffer;
  std::optional<std::string> latest;
  while (true)
  {
    const ssize_t size = ::recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
    if (size >= 0)
    {
      latest.emplace(buffer.data(), static_cast<std::size_t>(size));
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      return latest;
    }
    if (errno == EINTR)
    {
      continue;
    }
    return tl::make_unexpected(errnoMessage("recv()"));
  }
}

tl::expected<void, std::string> sendDatagram(const std::string& host, int port, const std::string& payload)
{
  if (payload.size() > kMaxUdpPayload)
  {
    return tl::make_unexpected("the payload has " + std::to_string(payload.size()) +
                               " bytes, more than one UDP datagram can carry (" + std::to_string(kMaxUdpPayload) + ")");
  }
  const auto addresses = resolve(host, port, false);
  if (!addresses)
  {
    return tl::make_unexpected(addresses.error());
  }
  std::string last_error = "no address to send to";
  for (const addrinfo* ai = addresses->get(); ai != nullptr; ai = ai->ai_next)
  {
    const int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (fd < 0)
    {
      last_error = errnoMessage("socket()");
      continue;
    }
    const ssize_t sent = ::sendto(fd, payload.data(), payload.size(), 0, ai->ai_addr, ai->ai_addrlen);
    const int send_errno = errno;
    ::close(fd);
    if (sent == static_cast<ssize_t>(payload.size()))
    {
      return {};
    }
    errno = send_errno;
    last_error = sent < 0 ? errnoMessage("sendto()") : "sendto() sent only part of the payload";
  }
  return tl::make_unexpected("cannot send UDP datagram to " + host + ":" + std::to_string(port) + ": " + last_error);
}
}  // namespace experimental_behaviors::json_utils
