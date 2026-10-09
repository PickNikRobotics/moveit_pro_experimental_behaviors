// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace experimental_behaviors::test
{
inline sockaddr_in loopback(int port)
{
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return addr;
}

/// Asks the kernel for a TCP port that is free right now.
inline int freeTcpPort()
{
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in addr = loopback(0);
  bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  socklen_t length = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &length);
  close(fd);
  return ntohs(addr.sin_port);
}

/// One request as the test server saw it. Header names are lower-case; repeated names are joined with ", ".
struct HttpRequest
{
  std::string method;
  std::string target;
  std::map<std::string, std::string> headers;
  std::string body;

  bool hasHeader(const std::string& name) const
  {
    return headers.count(name) > 0;
  }
};

/// The reply the test server sends for one request.
struct HttpReply
{
  int status = 200;
  std::vector<std::pair<std::string, std::string>> headers;
  std::string body;
  std::chrono::milliseconds delay{ 0 };
};

/// Minimal HTTP/1.1 server on the loopback interface. A handler chooses the reply for each request.
class TestHttpServer
{
public:
  using Handler = std::function<HttpReply(const HttpRequest&)>;

  explicit TestHttpServer(Handler handler) : handler_(std::move(handler))
  {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    const int enable = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in addr = loopback(0);
    bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t length = sizeof(addr);
    getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &length);
    port_ = ntohs(addr.sin_port);
    listen(listen_fd_, 4);
    thread_ = std::thread([this] { serve(); });
  }

  /// Answers every request with the same reply.
  explicit TestHttpServer(HttpReply reply) : TestHttpServer([reply](const HttpRequest&) { return reply; })
  {
  }

  ~TestHttpServer()
  {
    stop_ = true;
    shutdown(listen_fd_, SHUT_RDWR);
    close(listen_fd_);
    thread_.join();
  }

  std::string url(const std::string& path) const
  {
    return "http://127.0.0.1:" + std::to_string(port_) + path;
  }

  std::vector<HttpRequest> requests()
  {
    std::scoped_lock lock(mutex_);
    return requests_;
  }

private:
  void serve()
  {
    while (!stop_)
    {
      const int fd = accept(listen_fd_, nullptr, nullptr);
      if (fd < 0)
      {
        return;
      }
      handle(fd);
      close(fd);
    }
  }

  static std::string lower(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
  }

  void handle(int fd)
  {
    std::string data;
    char buffer[4096];
    std::size_t header_end = std::string::npos;
    while ((header_end = data.find("\r\n\r\n")) == std::string::npos)
    {
      const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0)
      {
        return;
      }
      data.append(buffer, static_cast<std::size_t>(n));
    }
    const std::string head = data.substr(0, header_end);
    HttpRequest request;
    request.method = head.substr(0, head.find(' '));
    const auto target_start = head.find(' ') + 1;
    request.target = head.substr(target_start, head.find(' ', target_start) - target_start);
    std::size_t content_length = 0;
    std::size_t line_start = head.find("\r\n");
    while (line_start != std::string::npos)
    {
      const std::size_t next = head.find("\r\n", line_start + 2);
      const std::string line =
          head.substr(line_start + 2, next == std::string::npos ? std::string::npos : next - line_start - 2);
      const auto colon = line.find(':');
      if (colon != std::string::npos)
      {
        const std::string name = lower(line.substr(0, colon));
        const auto value_start = line.find_first_not_of(' ', colon + 1);
        const std::string value = value_start == std::string::npos ? "" : line.substr(value_start);
        auto& slot = request.headers[name];
        slot = slot.empty() ? value : slot + ", " + value;
        if (name == "content-length")
        {
          content_length = std::stoul(value);
        }
      }
      line_start = next;
    }
    request.body = data.substr(header_end + 4);
    while (request.body.size() < content_length)
    {
      const ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0)
      {
        return;
      }
      request.body.append(buffer, static_cast<std::size_t>(n));
    }
    {
      std::scoped_lock lock(mutex_);
      requests_.push_back(request);
    }
    const HttpReply reply = handler_(request);
    const auto wake = std::chrono::steady_clock::now() + reply.delay;
    while (!stop_ && std::chrono::steady_clock::now() < wake)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::string response = "HTTP/1.1 " + std::to_string(reply.status) + " Test\r\n";
    for (const auto& [name, value] : reply.headers)
    {
      response += name + ": " + value + "\r\n";
    }
    response += "Content-Length: " + std::to_string(reply.body.size()) + "\r\nConnection: close\r\n\r\n" + reply.body;
    send(fd, response.data(), response.size(), MSG_NOSIGNAL);
  }

  Handler handler_;
  int listen_fd_ = -1;
  int port_ = 0;
  std::atomic<bool> stop_{ false };
  std::mutex mutex_;
  std::vector<HttpRequest> requests_;
  std::thread thread_;
};
}  // namespace experimental_behaviors::test
