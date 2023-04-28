#pragma once

#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>

#include "pileec2223_driver_laser_2d/SdpoDriverLaser2D.h"

namespace pileec2223_driver_laser_2d {

class SdpoDriverLaser2DROS {
 private:
  ros::NodeHandle nh;

  ros::Publisher pub_laser_;
  tf::TransformBroadcaster tf_broad_;

  ros::Time sample_time_;

  std::unique_ptr<SdpoDriverLaser2D> laser_;

  std::string model_;
  std::string serial_port_name_;
  int baud_rate_;
  std::string base_frame_id_;
  std::string laser_frame_id_;
  float laser_pose_x_;
  float laser_pose_y_;
  float laser_pose_z_;
  float laser_pose_yaw_;
  float laser_pose_pitch_;
  float laser_pose_roll_;
  float dist_min_;
  float dist_max_;
  float ang_min_;
  float ang_max_;

 public:
  SdpoDriverLaser2DROS();
  ~SdpoDriverLaser2DROS() = default;

  void start();

 private:
  void readParam();
  void pubLaserData();
};

} // namespace pileec2223_driver_laser_2d
