#include "socketcan.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <utility>

namespace damiao
{
SocketCAN::~SocketCAN()
{
  if (this->isOpen())
  {
    this->close();
  }
}

void SocketCAN::log_throttled_error(const std::string& interface_name) const
{
  static auto last_log_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - last_log_time;
  if (elapsed.count() >= 5.0)
  {
    std::cerr << "Unable to write: Socket " << interface_name << " not open" << std::endl;
    last_log_time = now;
  }
}

bool SocketCAN::open(const std::string& interface, boost::function<void(const can_frame& frame)> handler, int thread_priority)
{
  reception_handler = std::move(handler);
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock_fd_ == -1)
  {
    std::cerr << "[ERROR] Error: Unable to create a CAN socket" << std::endl;
    return false;
  }

  char name[16] = {};
  strncpy(name, interface.c_str(), interface.size());
  strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
  if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1)
  {
    std::cerr << "[ERROR] Unable to select CAN interface " << name << ": I/O control error" << std::endl;
    close();
    return false;
  }

  address_.can_family = AF_CAN;
  address_.can_ifindex = interface_request_.ifr_ifindex;
  int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
  if (rc == -1)
  {
    std::cerr << "[ERROR] Failed to bind socket to " << name << " network interface" << std::endl;
    close();
    return false;
  }
  return startReceiverThread(thread_priority);
}

void SocketCAN::close()
{
  terminate_receiver_thread_ = true;
  while (receiver_thread_running_)
  {
  }
  if (!isOpen())
  {
    return;
  }
  ::close(sock_fd_);
  sock_fd_ = -1;
}

bool SocketCAN::isOpen() const
{
  return sock_fd_ != -1;
}

void SocketCAN::write(can_frame* frame) const
{
  if (!isOpen())
  {
    log_throttled_error(interface_request_.ifr_name);
    return;
  }
  if (::write(sock_fd_, frame, sizeof(can_frame)) == -1)
  {
    log_throttled_error(interface_request_.ifr_name);
  }
}

static void* socketcan_receiver_thread(void* argv)
{
  auto* sock = static_cast<SocketCAN*>(argv);
  fd_set descriptors;
  int maxfd = sock->sock_fd_;
  struct timeval timeout{};
  can_frame rx_frame{};
  sock->receiver_thread_running_ = true;
  while (!sock->terminate_receiver_thread_)
  {
    timeout.tv_sec = 1;
    FD_ZERO(&descriptors);
    FD_SET(sock->sock_fd_, &descriptors);
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      size_t len = read(sock->sock_fd_, &rx_frame, CAN_MTU);
      if (len < 0)
      {
        continue;
      }
      if (sock->reception_handler != nullptr)
      {
        sock->reception_handler(rx_frame);
      }
    }
  }
  sock->receiver_thread_running_ = false;
  return nullptr;
}

bool SocketCAN::startReceiverThread(int thread_priority)
{
  terminate_receiver_thread_ = false;
  int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
  if (rc != 0)
  {
    std::cerr << "[ERROR] Unable to start receiver thread" << std::endl;
    return false;
  }
  sched_param sched{.sched_priority = thread_priority};
  pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched);
  return true;
}
}  // namespace damiao
