// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "adis_rcv_csv.h"

class ImuNodeRcvCsv {
public:
  explicit ImuNodeRcvCsv(ros::NodeHandle nh)
    : node_handle_(nh) {
    // Read parameters
    node_handle_.param("device", device_, std::string("/dev/ttyACM0"));
    node_handle_.param("frame_id", frame_id_, std::string("imu"));
    node_handle_.param("parent_id", parent_id_, std::string("base_link"));
    node_handle_.param("rate", rate_, 100.0);

    std::string mode_str = "Attitude";
    node_handle_.param("mode", mode_str, mode_str);
    if (mode_str == "Attitude") {
      imu_.SetMode(AdisRcvCsv::Mode::ATTIUDE);
    } else if (mode_str == "Register") {
      imu_.SetMode(AdisRcvCsv::Mode::REGISTER);
    } else {
      ROS_ERROR("Unknown mode [%s]. We use the default Attitude mode.", mode_str.c_str());
      imu_.SetMode(AdisRcvCsv::Mode::ATTIUDE);
      mode_str = "Attitude";
    }

    ROS_INFO("device: %s", device_.c_str());
    ROS_INFO("frame_id: %s", frame_id_.c_str());
    ROS_INFO("rate: %.1f [Hz]", rate_);
    ROS_INFO("mode: %s", mode_str.c_str());

    cant_rcv_cnt_ = 0;
    
    // Data publisher
    imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("data_raw", 100);
    updater_.add("imu", this, &ImuNodeRcvCsv::Diagnostic);
  }

  ~ImuNodeRcvCsv() {
    imu_.Close();
  }

  void Spin() {
    ros::Rate loop_rate(rate_);
    auto mode = imu_.GetMode();

    while (ros::ok()) {
      ros::spinOnce();
      switch (mode) {
        case AdisRcvCsv::Mode::REGISTER:
          UpdateAndPubRegMode();
          break;
        case AdisRcvCsv::Mode::ATTIUDE:
          UpdateAndPubYprMode();
          break;
        default:
          ROS_WARN("Unknown imu mode");
          break;
      }
      updater_.update();
      loop_rate.sleep();
    }

    imu_.Stop();
  }

  void Prepare() {
    while (ros::ok() && !imu_.Open(device_)) {
      ROS_WARN("Keep trying to open [%s] in 1 second period...", device_.c_str());
      ros::Duration(1).sleep();
    }

    while (ros::ok() && !imu_.Prepare()) {
      ROS_WARN("Keep trying to prepare the device in 1 second period...");
      ros::Duration(1).sleep();
    }

    updater_.setHardwareID(imu_.GetProductIdStr());
  }

private:
  AdisRcvCsv imu_;

  ros::NodeHandle node_handle_;
  ros::Publisher imu_data_pub_;
  diagnostic_updater::Updater updater_;

  std::string device_;
  std::string frame_id_;
  std::string parent_id_;

  double rate_;
  int cant_rcv_cnt_;
   
  void Diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (cant_rcv_cnt_ >= 1) {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                    "Data cannot be received for more than 1 second.");
    } else {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
  }


  void PubImuData() {
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    double acc[3];
    double gyro[3];
    imu_.GetAcc(acc);
    imu_.GetGyro(gyro);

    // Linear acceleration
    data.linear_acceleration.x = acc[0];
    data.linear_acceleration.y = acc[1];
    data.linear_acceleration.z = acc[2];

    // Angular velocity
    data.angular_velocity.x = gyro[0];
    data.angular_velocity.y = gyro[1];
    data.angular_velocity.z = gyro[2];

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    imu_data_pub_.publish(data);
  }

  void BroadcastImuPose() {
    static tf::TransformBroadcaster br;

    double ypr[3];
    imu_.GetYPR(ypr);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion q;
    q.setRPY(ypr[2]*DEG2RAD, ypr[1]*DEG2RAD, ypr[0]*DEG2RAD);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_id_, frame_id_));
  }

  void UpdateAndPubRegMode() {
    int res = imu_.UpdateRegMode();
    if (res == IMU_OK) {
      PubImuData();
      cant_rcv_cnt_ = 0;
    } else {
      if (res = IMU_ERR_CANT_RCV_DATA) cant_rcv_cnt_++;

      ROS_ERROR("Cannot update on reg mode");
    }
  }

  void UpdateAndPubYprMode() {
    int res = imu_.UpdateYprMode();
    if (res == IMU_OK) {
      BroadcastImuPose();
      cant_rcv_cnt_ = 0;
    } else {
      if (res = IMU_ERR_CANT_RCV_DATA) cant_rcv_cnt_++;

      ROS_ERROR("Cannot update on ypr mode");
    }
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh("~");

  ImuNodeRcvCsv node(nh);
  
  node.Prepare();
  node.Spin();
  return 0;
}
