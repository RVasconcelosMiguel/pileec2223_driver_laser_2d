#include "pileec2223_driver_laser_2d/SdpoDriverLaser2DROSCorrectData.h"

#include <exception>

#include "pileec2223_driver_laser_2d/utils.h"
#include "pileec2223_driver_laser_2d/RPLIDARS2.h"
#include "pileec2223_driver_laser_2d/YDLIDARX4.h"

namespace pileec2223_driver_laser_2d {

SdpoDriverLaser2DROSCorrectData::SdpoDriverLaser2DROSCorrectData() {
  try {
    readParam();
    laser_->setSerialPortParam(serial_port_name_, baud_rate_);
    laser_->setPubLaserData(
        std::bind(&SdpoDriverLaser2DROSCorrectData::pubLaserData, this));
    laser_->openSerial();
  } catch (std::exception& e) {
    ROS_FATAL("[pileec2223_driver_laser_2d] Error reading the node parameters (%s)",
              e.what());
    ros::shutdown();
  }

  pub_laser_ = nh.advertise<sensor_msgs::PointCloud>(
      "laser_scan_point_cloud", 1);
  sub_odom_ = nh.subscribe("odom", 1,
                           &SdpoDriverLaser2DROSCorrectData::subOdom, this);
}

void SdpoDriverLaser2DROSCorrectData::start() {
  laser_->start();
}

void SdpoDriverLaser2DROSCorrectData::readParam() {
  ros::NodeHandle nh_private("~");

  if (!nh_private.hasParam("model") || !nh_private.hasParam("baud_rate")) {
    throw std::runtime_error(
        "[SdpoDriverLaser2DROSCorrectData.cpp] SdpoDriverLaser2DROSCorrectData::readParam: "
        "the node pileec2223_driver_laser_2d_node requires the definition of the "
        "model and baud_rate parameters");
  }

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
        if (!nh_private.hasParam(param_name)) {
          ROS_INFO("[pileec2223_driver_laser_2d] Parameter %s not set in the "
                   "parameter server (using default value)",
                   param_name.c_str());
        }
      };

  nh_private.getParam("model", model_);
  ROS_INFO("[pileec2223_driver_laser_2d] Model of the 2D laser scanner: %s",
           model_.c_str());
  if (model_ == kSdpoDriverLaser2DYDLIDARXStr) {
    laser_.reset(new YDLIDARX4());
  } else if (model_ == kSdpoDriverLaser2DRPLIDARS2Str) {
    laser_.reset(new RPLIDARS2());
  } else {
    throw std::runtime_error(
        "[SdpoDriverLaser2DROSCorrectData.cpp] SdpoDriverLaser2DROSCorrectData::readParam: "
        "invalid 2D laser model (check documentation for supported ones)");
  }

  print_is_default_param_set("serial_port_name");
  nh_private.param<std::string>("serial_port_name", serial_port_name_,
                                "/dev/ttyUSB0");
  ROS_INFO("[pileec2223_driver_laser_2d] Serial port: %s", serial_port_name_.c_str());

  nh_private.getParam("baud_rate", baud_rate_);
  ROS_INFO("[pileec2223_driver_laser_2d] Baud rate: %d bps", baud_rate_);

  print_is_default_param_set("base_frame_id");
  nh_private.param<std::string>("base_frame_id", base_frame_id_,
                                "base_footprint");
  ROS_INFO("[pileec2223_driver_laser_2d] Base frame ID: %s", base_frame_id_.c_str());

  print_is_default_param_set("laser_frame_id");
  nh_private.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
  ROS_INFO("[pileec2223_driver_laser_2d] Laser frame ID: %s",
           laser_frame_id_.c_str());

  if(nh_private.hasParam("dist_min") && nh_private.hasParam("dist_max")) {
    nh_private.getParam("dist_min", dist_min_);
    nh_private.getParam("dist_max", dist_max_);
    ROS_INFO("[pileec2223_driver_laser_2d] Distance range: [%f, %f] m",
             dist_min_, dist_max_);
    if (dist_max_ <= dist_min_) {
      ROS_WARN("[pileec2223_driver_laser_2d] Distance range ignored "
               "(minimum must be greater than maximum)");
    } else {
      laser_->setDistRangeCheck(dist_min_, dist_max_);
    }
  } else if (nh_private.hasParam("dist_min") ||
      nh_private.hasParam("dist_max")) {
    ROS_WARN("[pileec2223_driver_laser_2d] Distance range ignored "
             "(both limits must be defined)");
  } else {
    ROS_INFO("[pileec2223_driver_laser_2d] Distance range not defined");
  }

  if(nh_private.hasParam("angle_min") && nh_private.hasParam("angle_max")) {
    nh_private.getParam("angle_min", ang_min_);
    nh_private.getParam("angle_max", ang_max_);
    ang_min_ = normAngRad(ang_min_ * M_PIf32 / 180.0f);
    ang_max_ = normAngRad(ang_max_ * M_PIf32 / 180.0f);
    if (ang_max_ < ang_min_) {
      std::swap(ang_min_, ang_max_);
    }
    ROS_INFO("[pileec2223_driver_laser_2d] Angle range: [%f, %f] deg",
             ang_min_ * 180.0f / M_PIf32, ang_max_ * 180.0f / M_PIf32);
    laser_->setAngRangeCheck(ang_min_, ang_max_);
  } else if (nh_private.hasParam("angle_min") ||
             nh_private.hasParam("angle_max")) {
    ROS_WARN("[pileec2223_driver_laser_2d] Angle range ignored "
             "(both limits must be defined)");
  } else {
    ROS_INFO("[pileec2223_driver_laser_2d] Angle range not defined");
  }
}

void SdpoDriverLaser2DROSCorrectData::pubLaserData() {
  sensor_msgs::PointCloud msg;

  // Debug message (remove it later...)
  ROS_INFO("[pileec2223_driver_laser_2d] Laser data (not corrected):");
  // just printing the first sample
  std::cout << laser_->dist_data[0] << " m @ "
            << laser_->ang_data[0] * 180 / M_PI << " deg "
            << std::endl;

  msg.header.frame_id = laser_frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.points.resize(laser_->data_count);

  float x_ini;
  float y_ini;
  float deltax;
  float deltay;
  float deltatheta;
  float deltat=1/360/8;


  for(size_t i = 0; i < laser_->data_count; i++) {

    deltax=vx*deltat;
    deltay=vy*deltat;
    deltatheta=vw*deltat;

    if(deltatheta==0){
      x=x+deltax*cos(theta)-deltay*sin(theta);
      y=y+deltax*sin(theta)+deltay*cos(theta);
    }else{
      x = x + (deltax * sin(theta + deltatheta) + deltay * (cos(theta + deltatheta) - 1)) * (cos(theta + deltatheta / 2) / deltatheta) - (deltax * (1 - cos(theta + deltatheta)) + deltay * sin(theta + deltatheta)) * (sin(theta + deltatheta / 2) / deltatheta);
      y = y + (deltax * sin(theta + deltatheta) + deltay * (cos(theta + deltatheta) - 1)) * (sin(theta + deltatheta / 2) / deltatheta) + (deltax * (1 - cos(theta + deltatheta)) + deltay * sin(theta + deltatheta)) * (cos(theta + deltatheta / 2) / deltatheta);
    }
    theta=theta+deltatheta;

    msg.points.at(i).x =
        laser_->dist_data[i] * cos(laser_->ang_data[i]);
    msg.points.at(i).y =
        laser_->dist_data[i] * sin(laser_->ang_data[i]);
    msg.points.at(i).z = 0;

    x_ini=msg.points.at(i).x;//
    y_ini=msg.points.at(i).y;

    msg.points.at(i).x = cos(theta) * x_ini - sin(theta) * y_ini + x;//
    msg.points.at(i).y = sin(theta) * x_ini + cos(theta) * y_ini + y;//

  }

  tf::StampedTransform laser2base_tf;
  laser2base_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  laser2base_tf.setRotation(tf::createQuaternionFromYaw(0.0));
  laser2base_tf.stamp_ = msg.header.stamp;
  laser2base_tf.frame_id_ = base_frame_id_;
  laser2base_tf.child_frame_id_ = laser_frame_id_;
  tf_broad_.sendTransform(laser2base_tf);

  pub_laser_.publish(msg);
}

void SdpoDriverLaser2DROSCorrectData::subOdom(const nav_msgs::Odometry& msg_odom) {
  // Process odom message (remove example debug message)
  ROS_INFO("[pileec2223_driver_laser_2d] Odom: %f m %f m (%f deg)",
           msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y,
           tf::getYaw(msg_odom.pose.pose.orientation) * 180.0 / M_PI);

    vx = msg_odom.twist.twist.linear.x;
    vy = msg_odom.twist.twist.linear.y;
    vw = msg_odom.twist.twist.angular.z;
        
  
  // Need to use later the oodometry velocity data for correcting laser data
  // Note: odom and laser are asynchronous from each other!!!!
}

void SdpoDriverLaser2DROSCorrectData::reset() {
  x=0;
  y=0;
  theta=0;
}

} // namespace pileec2223_driver_laser_2d