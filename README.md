## ADI_IMU_TR_Driver_ROS1

### Overview

TODO


### Connection

TODO

### How to use

#### Install
Go to your package directory and clone.
```
$ cd [your packages directory]
$ git clone --recursive https://github.com/technoroad/ADI_IMU_TR_Driver_ROS1
```

#### Build
Go to your workspace directory and run the build command.  
```
$ cd [your workspace directory]
$ colcon build --symlink-install
```
Then set the path.
```
$ source ./install/setup.bash
```

#### Run
Connect your sensor to USB port. Run the launch file as:

``` $ roslaunch adi_imu_tr_driver_ros1 adis_rcv_csv.launch ```

You can see the model of ADIS16470 breakout board in rviz panel.  

Then show imu values.
```
$ ros2 tocpic echo /imu/data_raw
```


### Topics
- /imu/data_raw (sensor_msgs/Imu)

  IMU raw output. It contains angular velocities and linear
  accelerations. The orientation is always unit quaternion.  
  example:

```
・・・
angular_velocity:
  x: -0.0116995596098
  y: -0.00314657808936
  z: 0.000579557116093
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.302349234658
  y: -0.303755252655
  z: 9.87837325989
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
・・・
```

- /diagnostics (diagnostic_msgs/DiagnosticArray)

  Sensor state output.  
  example:

```
・・・
header:
  seq: 80
  stamp:
    secs: 1587104853
    nsecs: 921894057
  frame_id: ''
status:
  -
    level: 0
    name: "adis_rcv_csv_node: imu"
    message: "OK"
    hardware_id: "ADIS16470"
    values: []
・・・
```
