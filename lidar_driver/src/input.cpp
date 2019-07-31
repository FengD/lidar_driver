/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#include "input.h"
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <string>

namespace itd_lidar {

namespace lidar_driver {
Input::Input(const std::string source_ip, const std::string multicast_ip, const int port)
  : source_ip_str_(source_ip),
    multicast_ip_str_(multicast_ip),
    port_(port),
    sockfd_(-1) {
  if (source_ip_str_.empty()) {
    printf("empty source ip\n");
    return;
  }
  inet_aton(source_ip_str_.c_str(), &source_ip_);
}

Input::~Input(void) {
  (void) close(sockfd_);
}

int Input::initParam() {
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    // Socket Error
    printf("socket\n");
    return SOCKETERR;
  }

  // my address information
  sockaddr_in my_addr;
  // initialize to zeros
  memset(&my_addr, 0, sizeof(my_addr));
  // host byte order
  my_addr.sin_family = AF_INET;
  // port in network byte order
  my_addr.sin_port = htons(port_);
  // automatically fill in my IP
  my_addr.sin_addr.s_addr = INADDR_ANY;
  // used for open multipule udp connnect
  int opt = 1;
  setsockopt( sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt) );

  if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
    // Bind Error
    printf("bind\n");
    return BINDERR;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
    printf("non-block\n");
    return NON_BLOCKERR;
  }

  if (!multicast_ip_str_.empty()) {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_str_.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if ( setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)) < 0) {
      printf("setsockopt\n");
      return SET_OPTIONERR;
    }
  }
  return NO_ERROR;
}

int Input::GetPacket(const int& packet_size, uint8_t *pkt) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    do {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      // poll() error?
      if (retval < 0) {
        if (errno != EINTR)
          printf("poll() error\n");
        return POLLERR;
      }
      // poll() timeout?
      if (retval == 0) {
        printf("poll() timeout\n");
        return TIMEOUTERR;
      }
      // device error?
      if ((fds[0].revents & POLLERR)
          || (fds[0].revents & POLLHUP)
          || (fds[0].revents & POLLNVAL)) {
        printf("poll() reports error\n");
        return DEVICEERR;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &pkt[0],
                              packet_size,  0,
                              (sockaddr*) &sender_address,
                              &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        printf("recvfail");
        return RECEIVEERR;
      }
    } else if ((size_t) nbytes == (size_t)packet_size) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (source_ip_str_ != "" && sender_address.sin_addr.s_addr != source_ip_.s_addr)
        continue;
      else
        break; // done
    }
  }
  return NO_ERROR;
}

int Input::GetPacketInnoviz(const int& packet_size1, const int& packet_size2, uint8_t *pkt) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    do {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      // poll() error?
      if (retval < 0) {
        if (errno != EINTR)
          printf("poll() error\n");
        return POLLERR;
      }
      // poll() timeout?
      if (retval == 0) {
        printf("poll() timeout\n");
        return TIMEOUTERR;
      }
      // device error?
      if ((fds[0].revents & POLLERR)
          || (fds[0].revents & POLLHUP)
          || (fds[0].revents & POLLNVAL)) {
        printf("poll() reports error\n");
        return DEVICEERR;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t nbytes = recvfrom(sockfd_, &pkt[0],
                              14000,  0,
                              (sockaddr*)(&sender_address),
                              &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        perror("recvfail");
        return RECEIVEERR;
      }
    } else if ((size_t) nbytes == (size_t)packet_size1 ||
               (size_t) nbytes == (size_t)packet_size2) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (source_ip_str_ != "" && sender_address.sin_addr.s_addr != source_ip_.s_addr)
        continue;
      else
        break;
    }
  }
  return NO_ERROR;
}

void Input::set_source_ip_str(const std::string& source_ip_str) {
  source_ip_str_ = source_ip_str;
}

void Input::set_multicast_ip_str(const std::string& multicast_ip_str) {
  multicast_ip_str_ = multicast_ip_str;
}

void Input::set_port(const int& port) {
  port_ = port;
}

} // namespace lidar_driver
} // namespace itd_lidar
