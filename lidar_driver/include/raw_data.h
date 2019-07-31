/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#ifndef _RAW_DATA_H_
#define _RAW_DATA_H_

#include <unistd.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>
#include <initializer_list>

#define FRAME_SIZE 17776

namespace itd_lidar {

namespace lidar_driver {
/**
* interface of the lidar data processing class
*/
class RawData {
 public:
  RawData() {}

  virtual ~RawData() {}

  virtual int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) = 0;

  virtual int getPointCloudTimestamp(uint8_t *data) = 0;

  virtual bool readCorrectionFile(std::initializer_list<std::string> path) = 0;

  // set reflect mode: single or dual
  virtual void setMode(int mode) = 0;
  // set lidar direction
  //                   1:x
  //                    *
  //              3:x *   * 4:x
  //                    *2:x
  //                    *
  virtual void setDirection(int direction) = 0;
  // set lidar version for RSlidar32 0: new version 1: old version
  virtual void setVersion(int version) = 0;
};

class RawDataRotate : public RawData {
 public:
  RawDataRotate() {}

  virtual ~RawDataRotate() {}

 protected:
  // count if the packet is the last packet of one cloud
  int packetCounter = 0;
  // the packets number of one pointcloud
  int packetsCloud = 0;

 private:
  virtual void analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) = 0;

  virtual void analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) = 0;
};

/**
* hesai pandar 40
*/
class RawDataP40P : public RawDataRotate {
 public:
  RawDataP40P();
  ~RawDataP40P();

  int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

  int getPointCloudTimestamp(uint8_t *data);

  bool readCorrectionFile(std::initializer_list<std::string> path);

  void setMode(int mode);

  void setDirection(int direction);

  void setVersion(int version);

 private:
  // defined by the User Guide
  static const int DATA_SIZE = 1240;
  static const int BYTE_PAR_BLOCK = 124;
  static const int CLOUD_PACKETS = 180; // single return mode by default
  static const int CLOUD_PACKETS_DUAL = 2 * CLOUD_PACKETS;
  static const int PACKET_BLOCKS = 10;
  static const int CHANNELS_NUM = 40;
  static const int POINTS_NUM = (PACKET_BLOCKS * CHANNELS_NUM);

  // default vertical angle
  float lidarLineAngle[CHANNELS_NUM] = {
  	7.00, 6.00, 5.00, 4.00, 3.00, 2.00,
  	1.67, 1.33, 1.00, 0.67, 0.33, 0.00, -0.33, -0.67,
  	-1.00, -1.33, -1.67, -2.00, -2.33, -2.67, -3.00, -3.33, -3.67,
  	-4.00, -4.33, -4.67, -5.00, -5.33, -5.67,
  	-6.00, -7.00, -8.00, -9.00, -10.0, -11.0, -12.0, -13.0, -14.0, -15.0, -16.0};

  // default horizontal angle correction
  float horizonCorrectAngle[CHANNELS_NUM] = {
  	0.0, 0.0, 0.0, 0.0, -2.5, -2.5, 2.5, -5.0,
  	-2.5, 2.5, -5.0, 2.5, 2.5, -5.0, 0.0, 2.5,
  	-5.0, 0.0, 5.0, -2.5, 0.0, 5.0, -2.5, 0.0,
  	5.0, -2.5, 2.5, 5.0, -2.5, 2.5, 2.5, 2.5,
  	0.0, 0.0, 0.0, 0.0, -2.5, -2.5, -2.5, -2.5
  };

  void analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter);

  void analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte);
};

/**
* velodyne vlp 16
*/
class RawDataVlp16 : public RawDataRotate {
 public:
  RawDataVlp16();
  ~RawDataVlp16();

  int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

  int getPointCloudTimestamp(uint8_t *data);

  bool readCorrectionFile(std::initializer_list<std::string> path);

  void setMode(int mode);

  void setDirection(int direction);

  void setVersion(int version);

 private:
  // defined by the User Guide
  static const int DATA_SIZE = 1200;
  static const int BYTE_PAR_BLOCK = 100;
  static const int CLOUD_PACKETS = 76; // single return mode by default
  static const int CLOUD_PACKETS_DUAL = 2 * CLOUD_PACKETS;
  static const int PACKET_BLOCKS = 12;
  static const int CHANNELS_NUM = 16;
  static const int POINTS_NUM = (2 * PACKET_BLOCKS * CHANNELS_NUM);

  float lidarLineAngle[CHANNELS_NUM] = {
    -15.00, 1.00, -13.00, 3.00, -11.00, 5.00,
    -9.00, 7.00, -7.00, 9.00, -5.00, 11.00, -3.00, 13.00,
    -1.00, 15.00};

  float ring[CHANNELS_NUM] = {
    0.00, 8.00, 1.00, 9.00, 2.00, 10.00,
    3.00, 11.00, 4.00, 12.00, 5.00, 13.00, 6.00, 14.00,
    7.00, 15.00};

  void analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter);

  void analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte);
};

/**
* rslidar 32 latest version 201903
*/
class RawDataRSL32 : public RawDataRotate {
 public:
  RawDataRSL32();
  ~RawDataRSL32();

  int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

  int getPointCloudTimestamp(uint8_t *data);

  bool readCorrectionFile(std::initializer_list<std::string> path);

  void setMode(int mode);

  void setDirection(int direction);

  void setVersion(int version);

 private:
  // defined by the User Guide
  static const int PACKET_HEADER_SIZE = 42;
  // static const int DATA_SIZE = 1240;
  static const int BYTE_PAR_BLOCK = 100;
  static const int CLOUD_PACKETS = 166;
  static const int PACKET_BLOCKS = 12;
  static const int CHANNELS_NUM = 32;
  static constexpr float RS32_DSR_TOFFSET = 3.0; // [us]
  static constexpr float RS32_BLOCK_TDURATION = 50.0; // [us]
  static constexpr float MAX_DISTANCE = 100.0f;
  static constexpr float MIN_DISTANCE = 0.2f;
  static constexpr float DISTANCE_RESOLUTION = 0.005f;
  static constexpr float DISTANCE_RESOLUTION_OLD = 0.01f;
  // static const int POINTS_NUM = (PACKET_BLOCKS * CHANNELS_NUM);
  // temp values
  float azimuth_diff_;
  int direction_ = 1;
  int dis_resolution_mode = 1; // Reserve for use

  // default vertical angle // NP19 left angle.csv
  float VERT_ANGLE[32] = {
    -10.333, -6.424, 2.333, 3.333, 4.667, 7, 10.333, 15.0167,
    0.333, 0, -0.2972, -0.667, 1.667, 1.333, 1, 0.6849,
    -24.9706, -14.6212, -7.9451, -5.4425, -3.667, -4, -4.333, -4.667,
    -2.3151, -2.667, -3, -3.2973, -1, -1.333, -1.6491, -1.9821};

  // default horizon angle // NP19 left angle.csv
  float HORI_ANGLE[32] = {
    7.4374, 7.69, 7.68, -8.11, 7.65, -8.33, 7.35, -8.55,
    -7.98, -2.61, 2.58, 7.75, -8.19, -2.95, 2.35, 7.51,
    -8.2457, -7.7189, -7.35, -7.42, -8.05, -2.75, 2.41, 7.65,
    -7.85, -2.61, 2.59, 7.82, -7.59, -2.35, 2.98, 8.08};

  // new version lidar channel sort
  float ring_new_version[CHANNELS_NUM] = {
    2.0, 4.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 21.0,
    20.0, 19.0, 18.0, 25.0, 24.0, 23.0, 22.0, 0.0, 1.0,
    3.0, 5.0, 9.0, 8.0, 7.0, 6.0, 13.0, 12.0, 11.0, 10.0,
    17.0, 16.0, 15.0, 14.0};

  // default ChannelNum
  int g_ChannelNum[32][51];

  // default CurveRate.csv
  float CurvesRate[32];

  // default curves.csv
  float aIntensityCal[7][32];

  void analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter);

  void analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte);
};

class InnovizPro : public RawData {
  struct vector3 {
 		float x; /*!< x axis */
 		float y; /*!< y axis */
 		float z; /*!< z axis */
 	};

 public:
  InnovizPro();

  ~InnovizPro();

  int unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data);

  int getPointCloudTimestamp(uint8_t *data);

  bool readCorrectionFile(std::initializer_list<std::string> path);

  void setMode(int mode);

  void setDirection(int direction);

  void setVersion(int version);

 private:
  int analysisSecondBlockHeader(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index);

  int analysisMeasurementBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index);

  void analysisMeasurementBlockPoints(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index, int firstIndex, int lastIndex);

  void analysisMeasurementBlockPoint(pcl::PointXYZI &point, uint8_t *data, int index, int directionIndex);

  vector3 directions[FRAME_SIZE];
};

} // namespace lidar_driver

} // namespace itd_lidar

#endif // _RAW_DATA_H_
