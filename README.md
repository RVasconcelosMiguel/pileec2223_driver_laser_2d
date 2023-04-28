# pileec2223_driver_laser_2d

This ROS package contains all the developments made in the course _Projeto_
_Integrador 2022/23_ of the Master of Science in Electrical and Computers
Engineering (ECE) at the Faculty of Engineering, University of Porto (FEUP).
The student responsible for this package is Rodrigo de Vasconcelos e Miguel
(up202008406@edu.fe.up.pt).

The project is a driver for communicating with
[RPLIDAR](https://www.slamtec.com/en) and variants (specifically,
[YDLIDAR](https://www.ydlidar.com/)) of 2D laser scanners. The main goal set for
the student is to correct the distortion in the laser's data due to the robot's
velocity in the odometric coordinate frame.

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [tf](https://wiki.ros.org/tf)

### Parameters

- model (`std::string`): model of the 2D laser (`"rplidars2" | "ydlidarx4"`)
- serial_port_name (`std::string = "/dev/ttyUSB0"`): name of the serial port
  - **Note:** do not forget to execute the following commands in the terminal:
    ```shell
    # Add user to group
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER

    # Update permissions of the serial port (a - all users + rw - read&write)
    sudo chmod a+rw /dev/ttyUSB0
    ```
- baud_rate (`unsigned int`): baud rate of the serial connection (bps)
  - Typical values of the baud rate for the supported laser scanners:
    - [YDLIDAR X4](https://www.ydlidar.com/products/view/5.html): `115200`
- base_frame_id (`std::string = "base_footprint"`): tf frame id of the robot
  base footprint coordinate frame
- laser_frame_id (`std::string = "laser"`): tf frame id of the 2D laser scanner
  coordinate frame
- dist_min (`float`): minimum distance range (m)
- dist_max (`float`): maximum distance range (m)
- angle_min (`float`): minimum angle range (deg, `[-180.0, 180.0[`)
- angle_max (`float`): maximum angle range (deg, `[-180.0, 180.0[`)
  - **Note:** if a minimum or a maximum value is set, both limits must be
    defined!

**correct_laser_data**

- TBC
- \<name\> (`<type> = <default value>`): \<description\> (\<units\>)

### Subscribes

**correct_laser_data**

- odom
  ([Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

### Publishes

- laser_scan_point_cloud
  ([PointCloud.msg](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git@github.com:INESC-TEC-MRDT/pileec2223_driver_laser_2d.git

# Build
cd ..
catkin build
```

### Launch

**no_correction**

_[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)_

```sh
roslaunch pileec2223_driver_laser_2d no_correction_YDLIDARX4.launch
```

_[RPLIDAR S2](https://www.slamtec.com/en/S2)_

```sh
roslaunch pileec2223_driver_laser_2d no_correction_RPLIDARS2.launch
```

**correct_laser_data**

_[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)_

```sh
roslaunch pileec2223_driver_laser_2d correct_laser_data_YDLIDARX4.launch
```

_[RPLIDAR S2](https://www.slamtec.com/en/S2)_

```sh
roslaunch pileec2223_driver_laser_2d correct_laser_data_RPLIDARS2.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
- Rodrigo de Vasconcelos e Miguel ([github](https://github.com/Rodrigomiguel2),
  [feup](mailto:up202008406@edu.fe.up.pt))
