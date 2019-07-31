#include <stdlib.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <initializer_list>
#include "driver.h"

pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

void addGroudInViewer() {
  pcl::PointCloud<pcl::PointXYZRGB> ground;
  pcl::PointXYZRGB p;
  float x = -50.0;
  float y = -50.0;
  for(int i = 0; i < 101; i++) {
    for(int j = 0; j < 101; j++) {
      p.x = x + i;
      p.y = y + j;
      p.z = 0.0;

      p.r = 255;
      p.g = 255;
      p.b = 255;

      if ((j == 50 || j == 49 || j == 51) && i >= 50) {
        p.r = 255;
        p.g = 0;
        p.b = 0;
      }

      if ((i == 50 || i == 49 || i == 51) && j >= 50) {
        p.r = 0;
        p.g = 255;
        p.b = 0;
      }
      ground.points.push_back(p);
    }
  }
  viewer.showCloud(ground.makeShared(), "ground");
}

void lidar_callback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp) {
  viewer.showCloud (cloud);
}

int main() {
  addGroudInViewer();
  // itd_lidar::lidar_driver::Driver driver("195.0.0.201", "", 2368, "VLP16", 0, "", lidar_callback);
  itd_lidar::lidar_driver::Driver driver("10.1.1.112", "", 8200, "InnovizPro", 0, "/home/ding/Hirain/itd_lidar_group/share/libs/lidar_driver/example/direction.csv", lidar_callback);
  // std::initializer_list<std::string> correctionfileList{
  //   "/home/wangxu/wangxu_work/2_PROJECT/Confirmed/Datong/NP24/src/ros_rslidar/rslidar_pointcloud/data/left/angle.csv",
  //   "/home/wangxu/wangxu_work/2_PROJECT/Confirmed/Datong/NP24/src/ros_rslidar/rslidar_pointcloud/data/left/ChannelNum.csv",
  //   "/home/wangxu/wangxu_work/2_PROJECT/Confirmed/Datong/NP24/src/ros_rslidar/rslidar_pointcloud/data/left/CurveRate.csv",
  //   "/home/wangxu/wangxu_work/2_PROJECT/Confirmed/Datong/NP24/src/ros_rslidar/rslidar_pointcloud/data/left/curves.csv"};
  // itd_lidar::lidar_driver::Driver driver("192.168.1.201", 6698, "RSL32", 0, 4, 1, correctionfileList, lidar_callback);
  driver.Start();
  while(1) {
    sleep(1000);
  }

  return 0;
}
