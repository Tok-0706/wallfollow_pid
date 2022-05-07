#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
#TODO kp,kd,ki
kp = -0.6
kd = -0.3
ki = 0

servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive' # 注意这里的topic是drive!

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) #TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.
        # TODO: implement
        # To calculate range for the -45 degrees and 225 degrees
        # -pi -> 0
        # -pi/2 = 0 degrees -> pi/2/0.005823 = 270
        # -45 degrees -> pi/4/0.005823 = 135 
        # 225 degrees -> 7*pi/4/0.005823 = 945
        index = (angle+45)*3+135
        return data.ranges[index]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0 # in radian
        #TODO: Use kp, ki & kd to implement a PID controller for 
        integral += prev_error
        angle = kp*error+ ki*integral+ kd*(error-prev_error)
        prev_error = error # Update the pre_error
        
        # calculate the velocity
        ratio = math.pi/180
        if 0<=angle and angle <10*ratio:
            velocity = 1.5 # meter
        elif 10*ratio<=angle and angle <20*ratio:
            velocity = 1 # meter
        else:
            velocity = 0.5
        
        velocity *= 2

        #print("Error ",error,"angle ",angle,"angle in degree", angle/ratio," velocity ",velocity)
        print("Error ",error)
        #velocity=0

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        # Follow left wall as per the algorithm 
        #TODO:implement
        # Calculate all the parameters
        theta = 45
        a,b = self.getRange(data,0),self.getRange(data,theta)
        alpha = math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
        Dt = b*math.cos(alpha)
        Dt1 = Dt + 0.5*math.sin(alpha) # D_t+1, L=0.5

        # Return the error, which is the difference between 'current' value and target value
        return Dt1-leftDist

    def lidar_callback(self, data):
        """ 
        """
        #TODO: replace with error returned by followLeft
        error = self.followLeft(data,DESIRED_DISTANCE_LEFT)
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)

