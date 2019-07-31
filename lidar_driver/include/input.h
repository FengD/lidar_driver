/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#ifndef _INPUT_H_
#define _INPUT_H_

#include <unistd.h>
#include <string>
#include <netinet/in.h>

namespace itd_lidar {

namespace lidar_driver {

enum StatusTable {
  SET_OPTIONERR = -8,
  NON_BLOCKERR = -7,
  BINDERR = -6,
  SOCKETERR = -5,
  RECEIVEERR = -4,
  TIMEOUTERR = -3,
  POLLERR = -2,
  DEVICEERR = -1,
  NO_ERROR = 0
};

class Input {
 public:
  Input(const std::string source_ip, const std::string multicast_ip, const int port);

  ~Input();

  int initParam();

  /**
   * @brief get the udp packets by given size
   * @param {int} size of the packet
   * @param {pointer} get the packets and put in the array
   */
  int GetPacket(const int& packet_size, uint8_t *pkt);

  /**
   * @brief get the udp packets by given size
   * @param {int} size of the packet1
   * @param {int} size of the packet2
   * @param {pointer} get the packets and put in the array
   */
  int GetPacketInnoviz(const int& packet_size1, const int& packet_size2, uint8_t *pkt);

  void set_source_ip_str(const std::string& source_ip_str);

  void set_multicast_ip_str(const std::string& multicast_ip_str);

  void set_port(const int& port);
 private:
  std::string source_ip_str_;
  std::string multicast_ip_str_;
  int port_;
  int sockfd_;
  in_addr source_ip_;
  // one second (in msec)
  static const int POLL_TIMEOUT = 1000;
};

} // namespace lidar_driver
} // namespace itd_lidar

#endif // _INPUT_H_
