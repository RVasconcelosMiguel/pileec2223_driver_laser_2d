#pragma once

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

#include "pileec2223_driver_laser_2d/SdpoDriverLaser2D.h"

namespace pileec2223_driver_laser_2d {

class SdpoDriverLaser2DROSCorrectData {
 private:
  ros::NodeHandle nh;

  ros::Publisher pub_laser_;
  ros::Publisher pub_laser_2;
  ros::Subscriber sub_odom_;
  tf::TransformBroadcaster tf_broad_;

  ros::Time sample_time_;

  std::unique_ptr<SdpoDriverLaser2D> laser_;

  std::string model_;
  std::string serial_port_name_;
  int baud_rate_;
  std::string base_frame_id_;
  std::string laser_frame_id_;
  float dist_min_;
  float dist_max_;
  float ang_min_;
  float ang_max_;


  float x;//
  float y;//
  float theta;//
  float vx;
  float vy;
  float vw;
  int id;
  int id2;

 public:
  SdpoDriverLaser2DROSCorrectData();
  ~SdpoDriverLaser2DROSCorrectData() = default;

  void start();

 private:
  void readParam();
  void pubLaserData();
  void subOdom(const nav_msgs::Odometry& msg_odom);
  void pubLaserDataNC();
};

} // namespace pileec2223_driver_laser_2d