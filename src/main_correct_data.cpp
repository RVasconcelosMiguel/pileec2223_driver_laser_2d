#include "pileec2223_driver_laser_2d/SdpoDriverLaser2DROSCorrectData.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "correct_laser_data");

  pileec2223_driver_laser_2d::SdpoDriverLaser2DROSCorrectData laser_driver;
  laser_driver.start();
  ros::spin();

  return 0;
}
