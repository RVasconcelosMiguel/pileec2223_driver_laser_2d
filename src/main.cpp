#include "pileec2223_driver_laser_2d/SdpoDriverLaser2DROS.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "no_correction");

  pileec2223_driver_laser_2d::SdpoDriverLaser2DROS laser_driver;
  laser_driver.start();
  ros::spin();

  return 0;
}
