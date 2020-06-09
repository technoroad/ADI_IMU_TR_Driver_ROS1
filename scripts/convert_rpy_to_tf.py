#!/usr/bin/env python
import rospy
import tf

import math
import serial
import time


def main():
    rospy.init_node('convert_rpy_to_tf', anonymous=True)

    # get parameters
    device = '/dev/ttyACM0'
    device = rospy.get_param('~device')
    parent_frame = "odom"
    parent_frame = rospy.get_param('~parent_frame')
    child_frame = "imu"
    child_frame = rospy.get_param('~child_frame')
    rospy.loginfo("device: " + device)
    rospy.loginfo("parent_frame: " + parent_frame)
    rospy.loginfo("child_frame: " + child_frame)
    
    ser = serial.Serial(device, 115200)

    br = tf.TransformBroadcaster()
    
    roll = 0
    pitch = 0
    yaw = 0
    
    pre_time = time.time()
    while not rospy.is_shutdown():
        var = ser.readline() 
        var = var.split(',')
        if len(var) == 3:
            yaw = float(var[0])
            pitch = float(var[1])
            roll = float(var[2])
        
#        print("roll: {}, pitch: {}, yaw: {}".format(roll, pitch, yaw))
        quaternion = tf.transformations.quaternion_from_euler(
                     math.radians(roll), math.radians(pitch), math.radians(yaw))
        br.sendTransform((0.0, 0.0, 0.0),
                         (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                          rospy.Time.now(),
                          child_frame,
                          parent_frame)

        now_time = time.time()
        elapsed_time = now_time - pre_time
        pre_time = now_time
#        print ("FPS: {0}".format(round(1/elapsed_time)))


if __name__ == '__main__':
    main()
