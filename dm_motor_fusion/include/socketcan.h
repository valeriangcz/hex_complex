#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>

#include <boost/function.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace damiao
{
class SocketCAN
{
private:
  ifreq interface_request_{};
  sockaddr_can address_{};
  pthread_t receiver_thread_id_{};

public:
  int sock_fd_ = -1;
  bool terminate_receiver_thread_ = false;
  bool receiver_thread_running_ = false;

  SocketCAN() = default;
  ~SocketCAN();

  void log_throttled_error(const std::string& interface_name) const;
  bool open(const std::string& interface, boost::function<void(const can_frame& frame)> handler, int thread_priority);
  void close();
  bool isOpen() const;
  void write(can_frame* frame) const;
  bool startReceiverThread(int thread_priority);

  boost::function<void(const can_frame& frame)> reception_handler;
};
}  // namespace damiao
