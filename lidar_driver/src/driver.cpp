/**
  * Copyright (C) 2019 Hirain Technologies
  * License: Modified BSD Software License Agreement
  * Author: Feng DING
  * Description: The driver which used to receive the udp packets and process the packets to point cloud
  */
#include "driver.h"
#include <string>
#include "input.h"
#include "raw_data.h"

namespace itd_lidar {

namespace lidar_driver {

  Driver::Driver(const std::string& source_ip,
                 const std::string& multicast_ip,
                 const int& data_port,
                 const std::string& model,
                 const int& mode,
                 const std::string& correction_file,
                 boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp)> lidar_callback) {
    std::initializer_list<std::string> correctionfileList{correction_file};
    new (this) Driver(source_ip, multicast_ip, data_port, model, mode, 0, 0, correctionfileList, lidar_callback);
  }

  Driver::Driver(const std::string& source_ip,
                 const std::string& multicast_ip,
                 const int& data_port,
                 const std::string& model,
                 const int& mode,
                 const int& direction,
                 const int& version,
                 std::initializer_list<std::string> correctionfileList,
                 boost::function<void(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp)> lidar_callback) {
    // init the udp input
    input_.reset(new Input(source_ip, multicast_ip, data_port));
    if (input_->initParam() != NO_ERROR) {
      return;
    }
    // init bool
    continue_lidar_recv_thread_ = false;
    continue_lidar_process_thread_ = false;
    // check the lidar model
    // and init the different data processing raw data
    lidar_model_ = model;
    if (lidar_model_ == "VLP16") {
      packet_size_ = 1206;
      frame_id_ = "velodyne";
      raw_data_.reset(new RawDataVlp16());
    } else if (lidar_model_ == "P40P") {
      packet_size_ = 1256;
      frame_id_ = "pandar";
      raw_data_.reset(new RawDataP40P());
    } else if (lidar_model_ == "RSL32") {
      packet_size_ = 1248;
      frame_id_ = "rslidar";
      raw_data_.reset(new RawDataRSL32());
      raw_data_->setDirection(direction);
      raw_data_->setVersion(version);
    } else if (lidar_model_ == "InnovizPro") {
      packet_size_ = 13488;
      frame_id_ = "innoviz";
      raw_data_.reset(new InnovizPro());
    }

    if (!raw_data_->readCorrectionFile(correctionfileList)) {
      printf("read correction file error\n");
      return;
    }
    // set the lidar mode(single / dual)
    raw_data_->setMode(mode);
    // init threads
    lidar_recv_thread_ = 0;
    lidar_process_thread_ = 0;
    // assignment callback
    user_lidar_callback_ = lidar_callback;
    // init semophore
    sem_init(&lidar_sem_, 0, 0);
    // init lock
    pthread_mutex_init(&lidar_lock_, NULL);
  }

  Driver::~Driver() {
    Stop();
  }

  void Driver::Start() {
    Stop();
    continue_lidar_recv_thread_ = true;
    continue_lidar_process_thread_ = true;
    lidar_recv_thread_ = new boost::thread(boost::bind(&Driver::LidarPacketRecvFn, this));
    lidar_process_thread_ = new boost::thread(boost::bind(&Driver::ProcessLidarPacketFn, this));
  }

  void Driver::Stop() {
    continue_lidar_recv_thread_ = false;
    continue_lidar_process_thread_ = false;

    if (lidar_process_thread_) {
    	lidar_process_thread_->join();
    	delete lidar_process_thread_;
    	lidar_process_thread_ = 0;
    }
    if (lidar_recv_thread_) {
    	lidar_recv_thread_->join();
    	delete lidar_recv_thread_;
    	lidar_recv_thread_ = 0;
    }
  }

  void Driver::LidarPacketRecvFn() {
    Packet p;
    if (lidar_model_ == "InnovizPro") {
      while (continue_lidar_recv_thread_) {
        int rc = input_->GetPacketInnoviz(packet_size_, 13448, p.data);
    		if (rc == NO_ERROR) {
    			PushLidarData(p);
    		}
      }
    } else {
      while (continue_lidar_recv_thread_) {
        int rc = input_->GetPacket(packet_size_, p.data);
    		if (rc == NO_ERROR) {
    			PushLidarData(p);
    		}
      }
    }
  }

  void Driver::PushLidarData(const Packet& pkt) {
  	pthread_mutex_lock(&lidar_lock_);
  	lidar_packet_list_.push_back(pkt);
  	if (lidar_packet_list_.size() > 0) {
  		sem_post(&lidar_sem_);
  	}
  	pthread_mutex_unlock(&lidar_lock_);
  }

  void Driver::ProcessLidarPacketFn() {
  	int last_timestamp = 0;
  	struct timespec ts;

		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> outMsg(new pcl::PointCloud<pcl::PointXYZI>());
  	while (continue_lidar_process_thread_) {
  		if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
  			printf("get time error\n");
  		}

  		ts.tv_sec += 1;
  		if (sem_timedwait(&lidar_sem_, &ts) == -1) {
  			continue;
  		}
  		pthread_mutex_lock(&lidar_lock_);
      Packet packet = lidar_packet_list_.front();
      lidar_packet_list_.pop_front();
  		pthread_mutex_unlock(&lidar_lock_);

      int ret = raw_data_->unpack(*outMsg, packet.data);
      last_timestamp = raw_data_->getPointCloudTimestamp(packet.data);
  		outMsg->header.frame_id = frame_id_;
  		outMsg->height = 1;
      if (ret == 1 && outMsg->points.size() > 0) {
        if (user_lidar_callback_) {
          user_lidar_callback_(outMsg, last_timestamp);
        }
        outMsg->clear();
      }
  	}
  }
} // namespace lidar_driver

} // namespace itd_lidar
