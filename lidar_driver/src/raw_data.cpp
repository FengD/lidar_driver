/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: This file used to define the udp input
  */
#include "raw_data.h"
#include <limits>
#include <string>

static int getUint32 (uint8_t *data, int startIndex) {
  return (data[3 + startIndex]<<24) + (data[2 + startIndex]<<16) + (data[1 + startIndex]<<8) + (data[startIndex]);
}

static int getUint16 (uint8_t *data, int startIndex) {
  return (data[1 + startIndex]<<8) + (data[startIndex]);
}

static int getUint8 (uint8_t *data, int startIndex) {
  return (data[startIndex]);
}

namespace itd_lidar {

namespace lidar_driver {
  RawDataP40P::RawDataP40P() : RawDataRotate() { }

  RawDataP40P::~RawDataP40P() { }

  int RawDataP40P::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    packetCounter++;
    for (int i = 0; i < PACKET_BLOCKS; i++) {
  		analysisBlock(cloud, data, i);
    }
    if (packetCounter % packetsCloud == 0) {
      packetCounter = 0;
      return 1;
    } else {
      return 0;
    }
  }

  int RawDataP40P::getPointCloudTimestamp(uint8_t *data) {
    int timestamp = ((data[1250]) + (data[1251] << 8) + (data[1252] << 16) + (data[1253] << 24));
    return timestamp;
  }

  bool RawDataP40P::readCorrectionFile(std::initializer_list<std::string> path) {
    std::string path_c = *path.begin();
    boost::filesystem::path filePath(path_c);
    if (!boost::filesystem::is_regular(filePath)) {
      printf("invalid lidar correction file, use default values\n");
      return false;
    }

    std::ifstream ifs(filePath.string());
    if (!ifs.is_open())
      return false;

    std::string line;
    if (std::getline(ifs, line)) { // first line "Laser id,Elevation,Azimuth"
      printf("parsing correction file.\n");
    }

    int lineCounter = 0;
    while (std::getline(ifs, line)) {
      if (lineCounter++ >= 40)
          break;

      int lineId = 0;
      double elev, azimuth;

      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;
      lineId--;
      lidarLineAngle[lineId] = elev;
      horizonCorrectAngle[lineId] = azimuth;
    }
    return true;
  }

  void RawDataP40P::setMode(int mode) {
    if (mode == 0) {
      packetsCloud = CLOUD_PACKETS;
    } else if (mode == 1) {
      packetsCloud = CLOUD_PACKETS_DUAL;
    }
  }

  void RawDataP40P::setDirection(int direction) {
    (void) direction;
    return;
  }

  void RawDataP40P::setVersion(int version) {
    (void) version;
    return;
  }

  void RawDataP40P::analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) {
    // calculate azimuth (angle) by udp data byte
    float azimuth = (data[blockCounter * BYTE_PAR_BLOCK + 2] + (data[blockCounter * BYTE_PAR_BLOCK + 3] << 8)) / 100.0;

    pcl::PointXYZI p;
    for (int j = 0; j < CHANNELS_NUM; j++) {
      analysisChannel(p, data, j, blockCounter, azimuth, 4);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void RawDataP40P::analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) {
    float angle_horizontal_deg = azimuth + horizonCorrectAngle[channelCounter];
    // add correct value for horizontal angle and transform to arc
    float angle_horizontal = angle_horizontal_deg * M_PI / 180.0;
    // calculate sinus
    float angle_horizontal_sin = sin(angle_horizontal);
    // calculate cosus
    float angle_horizontal_cos = sqrt(1 - angle_horizontal_sin * angle_horizontal_sin);
    if (angle_horizontal > M_PI / 2 && angle_horizontal < 3 * M_PI / 2) {
    	angle_horizontal_cos = -angle_horizontal_cos;
    }
    // the index of each start byte position of each channel in one block
    int index = startByte + channelCounter * 3 + blockCounter * BYTE_PAR_BLOCK;
    // decode the distance value from udp
    float distance = (data[index] + (data[index + 1] << 8)) * 0.004;
    // decode the intensity value from udp
    float intensity = (data[index + 2]);
    // distance project on xy plane
    float distance2xyPlane = 0.0;
    // distance project on z
    float distance2z = 0.0;
    // cos and sin of the offset angle
    float angle_vertical_cos = cos(lidarLineAngle[channelCounter] * M_PI / 180.0);
    float angle_vertical_sin = sin(lidarLineAngle[channelCounter] * M_PI / 180.0);

    distance2xyPlane = distance * angle_vertical_cos;
    distance2z = distance * angle_vertical_sin;
    // calculate the x y z of one point
    float x_lidar_coordinate = distance2xyPlane * angle_horizontal_sin;
    float y_lidar_coordinate = distance2xyPlane * angle_horizontal_cos;
    float z_lidar_coordinate = distance2z;
    // the direction of the angle zero is the direction of the wire
    point.x = -y_lidar_coordinate;
    point.y = x_lidar_coordinate;
    point.z = z_lidar_coordinate;
    point.intensity = intensity;
    point.data_c[1] = angle_horizontal_deg;
    point.data_c[2] = channelCounter;
    point.data_c[3] = lidarLineAngle[channelCounter];

    if (point.x == 0 && point.y == 0 && point.z == 0) {
      point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  RawDataVlp16::RawDataVlp16() : RawDataRotate() { }

  RawDataVlp16::~RawDataVlp16() { }

  int RawDataVlp16::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    packetCounter++;
    for (int i = 0; i < PACKET_BLOCKS; i++) {
  		analysisBlock(cloud, data, i);
    }
    if (packetCounter % packetsCloud == 0) {
      packetCounter = 0;
      return 1;
    } else {
      return 0;
    }
  }

  int RawDataVlp16::getPointCloudTimestamp(uint8_t *data) {
    int timestamp = ((data[1200]) + (data[1201] << 8) + (data[1202] << 16) + (data[1203] << 24));
    return timestamp;
  }

  bool RawDataVlp16::readCorrectionFile(std::initializer_list<std::string> path) {
    (void)path;
    printf("no need for vlp16\n");
    return true;
  }

  void RawDataVlp16::setMode(int mode) {
    if (mode == 0) {
      packetsCloud = CLOUD_PACKETS;
    } else if (mode == 1) {
      packetsCloud = CLOUD_PACKETS_DUAL;
    }
  }

  void RawDataVlp16::setDirection(int direction) {
    (void) direction;
    return;
  }

  void RawDataVlp16::setVersion(int version) {
    (void) version;
    return;
  }

  void RawDataVlp16::analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) {
    // calculate azimuth (angle) by udp data byte
    float azimuth = (data[blockCounter * BYTE_PAR_BLOCK + 2] + (data[blockCounter * BYTE_PAR_BLOCK + 3] << 8)) / 100.0;
    pcl::PointXYZI p;
    for (int j = 0; j < CHANNELS_NUM; j++) {
      analysisChannel(p, data, j, blockCounter, azimuth, 4);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }

      analysisChannel(p, data, j, blockCounter, (azimuth + 0.2), 52);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void RawDataVlp16::analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) {
    // add correct value for horizontal angle and transform to arc
    float angle_horizontal = azimuth * M_PI / 180.0;
    // calculate sinus
    float angle_horizontal_sin = sin(angle_horizontal);
    // calculate cosus
    float angle_horizontal_cos = sqrt(1 - angle_horizontal_sin * angle_horizontal_sin);
    if (angle_horizontal > M_PI / 2 && angle_horizontal < 3 * M_PI / 2) {
    	angle_horizontal_cos = -angle_horizontal_cos;
    }
    // the index of each start byte position of each channel in one block
    int index = startByte + channelCounter * 3 + blockCounter * BYTE_PAR_BLOCK;
    // decode the distance value from udp
    float distance = (data[index] + (data[index + 1] << 8)) * 0.002;
    // decode the intensity value from udp
    float intensity = (data[index + 2]);
    // distance project on xy plane
    float distance2xyPlane = 0.0;
    // distance project on z
    float distance2z = 0.0;
    // cos and sin of the offset angle
    float angle_vertical_cos = cos(lidarLineAngle[channelCounter] * M_PI / 180.0);
    float angle_vertical_sin = sin(lidarLineAngle[channelCounter] * M_PI / 180.0);

    distance2xyPlane = distance * angle_vertical_cos;
    distance2z = distance * angle_vertical_sin;
    // calculate the x y z of one point
    float x_lidar_coordinate = distance2xyPlane * angle_horizontal_sin;
    float y_lidar_coordinate = distance2xyPlane * angle_horizontal_cos;
    float z_lidar_coordinate = distance2z;
    // the direction of the angle zero is the opposite direction of the wire
    point.x = y_lidar_coordinate;
    point.y = -x_lidar_coordinate;
    point.z = z_lidar_coordinate;
    point.intensity = intensity;
    point.data_c[1] = azimuth;
    point.data_c[2] = ring[channelCounter];
    point.data_c[3] = lidarLineAngle[channelCounter];

    if (point.x == 0 && point.y == 0 && point.z == 0) {
      point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  RawDataRSL32::RawDataRSL32() : RawDataRotate() { }

  RawDataRSL32::~RawDataRSL32() { }

  bool RawDataRSL32::readCorrectionFile(std::initializer_list<std::string> path) {  // angle.csv ChannelNum.csv CurveRate.csv curves.csv  // don't include limit.csv
    if (path.size() != 4) {
      printf("Wrong input file number. RSlidar 32 use 5 correction file. Using default file.\n");
      return false;
    }

    std::string path_c;
    int counter = 0;
    for (auto pt = path.begin(); pt != path.end(); ++pt) {
      counter++;
      path_c = *pt;
      boost::filesystem::path filePath(path_c);
      if (!boost::filesystem::is_regular(filePath)) {
        printf("invalid lidar correction file, use default values\n");
        return false;
      }

      if (counter == 1) { // angle.csv
        std::ifstream ifs(filePath.string());
        if (!ifs.is_open())
          return false;

        std::string line;
        printf("parsing angle correction file.\n");

        int lineCounter = 0;
        while (std::getline(ifs, line)) {
          if (lineCounter++ >= 32)
            break;

          double vet_a, hor_a;

          std::stringstream ss(line);
          std::string subline;
          std::getline(ss, subline, ',');
          std::stringstream(subline) >> vet_a;
          std::getline(ss, subline, ',');
          std::stringstream(subline) >> hor_a;
          VERT_ANGLE[lineCounter-1] = vet_a;
          HORI_ANGLE[lineCounter-1] = hor_a;
        }
      } else if (counter == 2){ // ChannelNum.csv
        std::ifstream ifs(filePath.string());
        if (!ifs.is_open())
          return false;

        std::string line;
        printf("parsing ChannelNum correction file.\n");

        int lineCounter = 0;
        while (std::getline(ifs, line)) {
          if (lineCounter++ >= 32)
              break;

          std::stringstream ss(line);
          std::string subline;
          double g_c;
          for(size_t i = 0; i < 51; i++)
          {
            std::getline(ss, subline, ',');
            std::stringstream(subline) >> g_c;
            g_ChannelNum[lineCounter-1][i] = g_c;
          }
        }
      } else if (counter == 3){ //  CurveRate.csv
        std::ifstream ifs(filePath.string());
        if (!ifs.is_open())
          return false;

        std::string line;
        printf("parsing CurveRate correction file.\n");

        int lineCounter = 0;
        while (std::getline(ifs, line)) {
          if (lineCounter++ >= 32)
            break;

          std::stringstream ss(line);
          std::string subline;
          double curve_c;
          std::getline(ss, subline, ',');
          std::stringstream(subline) >> curve_c;
          CurvesRate[lineCounter-1] = curve_c;
        }
      } else if (counter == 4) {  // curves.csv
        std::ifstream ifs(filePath.string());
        if (!ifs.is_open())
          return false;

        std::string line;
        printf("parsing curves correction file.\n");

        int lineCounter = 0;
        while (std::getline(ifs, line)) {
          if (lineCounter++ >= 7)
              break;

          std::stringstream ss(line);
          std::string subline;
          double a_c;
          for(size_t i = 0; i < 32; i++)
          {
            std::getline(ss, subline, ',');
            std::stringstream(subline) >> a_c;
            aIntensityCal[lineCounter-1][i] = a_c;
          }
        }
      } else {
        return false;
      }
    }
    return true;
  }

  void RawDataRSL32::setMode(int mode) {  // RSlidar 32 only have single model   no dual mode
    (void) mode;
    packetsCloud = CLOUD_PACKETS;
    return;
  }

  void RawDataRSL32::setDirection(int direction) {
    direction_ = direction;
  }

  void RawDataRSL32::setVersion(int version) {
    if (version == 0) { // new version
      dis_resolution_mode = 0;
    } else {  // old version
      dis_resolution_mode = 1;
    }

    return;
  }

  int RawDataRSL32::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    packetCounter++;
    for(size_t i = 0; i < PACKET_BLOCKS; i++) {
      analysisBlock(cloud, data, i);
      // if (azimuth_diff_ <= 0.0 || azimuth_diff_ > 0.36) {
      //   continue;
      // }
    }
    if (packetCounter % packetsCloud == 0) {
      packetCounter = 0;
      return 1;
    } else {
      return 0;
    }
  }

  int RawDataRSL32::getPointCloudTimestamp(uint8_t *data) {
    int timestamp = (((data[26]) + (data[27] << 8)) * 1000 + ((data[28]) + (data[29] << 8)));
    return timestamp;
  }

  void RawDataRSL32::analysisBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int blockCounter) {
    // calculate azimuth (angle) by udp data byte
    azimuth_diff_ = 0;
    float azimuth = ((data[PACKET_HEADER_SIZE + blockCounter * BYTE_PAR_BLOCK + 2] << 8) + data[PACKET_HEADER_SIZE + blockCounter * BYTE_PAR_BLOCK + 3]) / 100.0;
    if (blockCounter < 11) {
      float azimuth_next = ((data[PACKET_HEADER_SIZE + (blockCounter + 1) * BYTE_PAR_BLOCK + 2] << 8) + data[PACKET_HEADER_SIZE + (blockCounter + 1) * BYTE_PAR_BLOCK + 3]) / 100.0;
      azimuth_diff_ = azimuth_next - azimuth;
    } else {
      float azimuth_before = ((data[PACKET_HEADER_SIZE + (blockCounter - 1) * BYTE_PAR_BLOCK + 2] << 8) + data[PACKET_HEADER_SIZE + (blockCounter - 1) * BYTE_PAR_BLOCK + 3]) / 100.0;
      azimuth_diff_ = azimuth - azimuth_before;
    }
    // ignore azimuth change abnormal
    // if (azimuth_diff_ <= 0.0 || azimuth_diff_ > 0.36) {
    //   std::cout<<"azimuth change abnormal. Block dropped."<<std::endl;
    //   return;
    // }
    pcl::PointXYZI p;
    for(size_t j = 0; j < CHANNELS_NUM; j++) {
      analysisChannel(p, data, j, blockCounter, azimuth, 4);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void RawDataRSL32::analysisChannel(pcl::PointXYZI &point, uint8_t *data, int channelCounter, int blockCounter, float azimuth, int startByte) {
    // correct azimuth according to firing time : 50us between two block
    int dsr_temp = (channelCounter >= 16) ? channelCounter-16 : channelCounter;
    float azimuth_corrected_f = azimuth + (azimuth_diff_ * (dsr_temp * RS32_DSR_TOFFSET) / RS32_BLOCK_TDURATION);
    float azimuth_corrected = azimuth_corrected_f + HORI_ANGLE[channelCounter];

    int index = PACKET_HEADER_SIZE + startByte + channelCounter * 3 + blockCounter * BYTE_PAR_BLOCK;
    if (dis_resolution_mode == 0) { // new version  no AB packet
      // read distance
      float distance = (data[index + 1] + ((data[index]&0x7f) * 256)) * DISTANCE_RESOLUTION;

      // temperature compensate set Temp as 50
      int distance_comp = g_ChannelNum[channelCounter][50];
      if (distance <= distance_comp * DISTANCE_RESOLUTION) {
        distance = 0.0;
      } else {
        distance = distance - distance_comp * DISTANCE_RESOLUTION;
      }

      // read intensity
      float intensity = data[index + 2];

      float arg_vert = VERT_ANGLE[channelCounter] / 180.0 * M_PI;
      float arg_horiz = azimuth_corrected / 180.0 * M_PI;
      if (distance > MAX_DISTANCE || distance < MIN_DISTANCE) { // invalid distance
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        point.intensity = 0;
        return;
      } else {
        switch (direction_)
        {
          case 1: // opposite side of cable
            point.x = distance * cos(arg_vert) * cos(arg_horiz);
            point.y = -distance * cos(arg_vert) * sin(arg_horiz);
            break;

          case 2: // same side of cable
            point.x = -distance * cos(arg_vert) * cos(arg_horiz);
            point.y = distance * cos(arg_vert) * sin(arg_horiz);
            break;

          case 3: // left side of cable
            point.x = -distance * cos(arg_vert) * sin(arg_horiz);
            point.y = -distance * cos(arg_vert) * cos(arg_horiz);
            break;

          case 4: // left side of cable
            point.x = distance * cos(arg_vert) * sin(arg_horiz);
            point.y = distance * cos(arg_vert) * cos(arg_horiz);
            break;
        }
        point.z = distance * sin(arg_vert);
        point.intensity = intensity;
        point.data_c[1] = azimuth;
        point.data_c[2] = ring_new_version[channelCounter];
        point.data_c[3] = VERT_ANGLE[channelCounter];
      }
    } else {  // AB packet old version
      int index_4flag = PACKET_HEADER_SIZE + startByte + blockCounter * BYTE_PAR_BLOCK;
      int ABflag = 0;
      if ((data[index_4flag]&128) != 0) {
        ABflag = 1;
      }

      if (ABflag == 1 && channelCounter < 16) {
        index = index + 48;
      } else if (ABflag == 1 && channelCounter >= 16) {
        index = index - 48;
      }

      // read distance
      float distance = (data[index + 1] + ((data[index]&0x7f) * 256)) * DISTANCE_RESOLUTION_OLD;
      // temperature compensate set Temp as 50
      int distance_comp = g_ChannelNum[channelCounter][50];
      if (distance <= distance_comp * DISTANCE_RESOLUTION_OLD) {
        distance = 0.0;
      } else {
        distance = distance - distance_comp * DISTANCE_RESOLUTION_OLD;
      }

      // read intensity
      float intensity = data[index + 2];

      float arg_vert = VERT_ANGLE[channelCounter] / 180.0 * M_PI;
      float arg_horiz = azimuth_corrected / 180.0 * M_PI;
      if (distance > MAX_DISTANCE || distance < MIN_DISTANCE) { // invalid distance
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        point.intensity = 0;
        return;
      } else {
        switch (direction_)
        {
          case 1: // opposite side of cable
            point.x = distance * cos(arg_vert) * cos(arg_horiz);
            point.y = -distance * cos(arg_vert) * sin(arg_horiz);
            break;

          case 2: // same side of cable
            point.x = -distance * cos(arg_vert) * cos(arg_horiz);
            point.y = distance * cos(arg_vert) * sin(arg_horiz);
            break;

          case 3: // right side of cable
            point.x = -distance * cos(arg_vert) * sin(arg_horiz);
            point.y = -distance * cos(arg_vert) * cos(arg_horiz);
            break;

          case 4: // left side of cable
            point.x = distance * cos(arg_vert) * sin(arg_horiz);
            point.y = distance * cos(arg_vert) * cos(arg_horiz);
            break;
        }
        point.z = distance * sin(arg_vert);
        point.intensity = intensity;
        point.data_c[1] = azimuth;
        point.data_c[2] = ring_new_version[channelCounter]; //TODO: need to be revised according to old version
        point.data_c[3] = VERT_ANGLE[channelCounter];
      }
    }
  }

  // =================================Innoviz========================================
  InnovizPro::InnovizPro() : RawData() { }

  InnovizPro::~InnovizPro() { }

  bool InnovizPro::readCorrectionFile(std::initializer_list<std::string> path) {
    std::string path_c = *path.begin();
    // std::cout << path_c<<std::endl;
    boost::filesystem::path filePath(path_c);
    // if (!boost::filesystem::is_regular(filePath)) {
    //   printf("invalid lidar correction file, error\n");
    //   return false;
    // }

    std::ifstream ifs(filePath.string());
    if (!ifs.is_open())
      return false;

    std::string line;
    int id;
    while (std::getline(ifs, line)) {
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> id;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> directions[id].x;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> directions[id].y;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> directions[id].z;
    }
    return true;
  }

  int InnovizPro::unpack(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data) {
    return analysisSecondBlockHeader(cloud, data, 76);
  }

  int InnovizPro::analysisSecondBlockHeader(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index) {
    int marker = getUint32(data, index);
    int blockLength = getUint32(data, index + 4);
    int blockType = getUint32(data, index + 8);
    if (blockType == 1) {
      return analysisMeasurementBlock(cloud, data, index + 40);
    } else if (blockType == 2) {
      return analysisMeasurementBlock(cloud, data, index);
    }
    return -1;
  }

  int InnovizPro::analysisMeasurementBlock(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index) {
    int frame = getUint16(data, 24 + index);
    int firstIndex = getUint32(data, 28 + index);
    int lastIndex = getUint32(data, 32 + index);
    analysisMeasurementBlockPoints(cloud, data, 36 + index, firstIndex, lastIndex);
    if (frame - lastIndex != 1) {
      return -1;
    } else {
      return 1;
    }
  }

  void InnovizPro::analysisMeasurementBlockPoints(pcl::PointCloud<pcl::PointXYZI> &cloud, uint8_t *data, int index, int firstIndex, int lastIndex) {
    pcl::PointXYZI p;
    int endIndex = lastIndex - firstIndex + 1;

    for (int j = 0; j < endIndex; j++) {
      analysisMeasurementBlockPoint(p, data, index + j * 3, firstIndex + j);
      if (!pcl_isnan(p.x) && !pcl_isnan(p.y) && !pcl_isnan(p.z)) {
        cloud.push_back(p);
      }
    }
  }

  void InnovizPro::analysisMeasurementBlockPoint(pcl::PointXYZI &point, uint8_t *data, int index, int directionIndex) {
    // decode the distance value from udp
    float distance = ((data[index]) + (data[index + 1] << 8)) * 0.01;
    // decode the intensity value from udp
    float intensity = (data[index + 2]);

    point.x = directions[directionIndex].x * distance;
    point.y = directions[directionIndex].y * distance;
    point.z = directions[directionIndex].z * distance;
    point.intensity = intensity;
    point.data_c[1] = directionIndex;
    point.data_c[2] = distance;

    if (point.x == 0 && point.y == 0 && point.z == 0) {
      point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  void InnovizPro::setMode(int mode) {
    (void) mode;
    return;
  }

  void InnovizPro::setDirection(int direction) {
    (void) direction;
    return;
  }

  void InnovizPro::setVersion(int version) {
    (void) version;
    return;
  }

  int InnovizPro::getPointCloudTimestamp(uint8_t *data) {
    // TODO
    return 0;
  }

} // namespace lidar_driver

} // namespace itd_lidar
