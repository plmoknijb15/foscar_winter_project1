#!/usr/bin/python

import cv2, rospy, time
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from turtlesim.msg import Pose

bridge = CvBridge()
img = np.empty(shape=[0])

x, y, theta = 0, 0, 0
lower_blue = (100,50, 50)
upper_blue = (140, 255, 255)

top_left = (2.5, 8.5)
middle_left = (2.5, 5.5)
bottom_left = (2.5, 2.5)
top_middle = (5.5, 8.5)
center = (5.5, 5.5)
bottom_middle = (5.5, 2.5)
top_right = (8.5, 8.5)
middle_top = (8.5, 5.5)
bottom_right = (8.5, 2.5)

angle_check = 0.1
distacne_check = 0.1
detect_threshold = 5000

control_msg = Twist()

def image_callback(img_data):
    global bridge
    global img
    img = bridge.imgmsg_to_cv2(img_data, "bgr8")

def pose_callback(pose_data):
    global x
    global y
    global theta
    x = pose_data.x
    y = pose_data.y
    theta = pose_data.theta

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def control_msg_publish(linear_x, angular_z):
    	control_msg.linear.x = linear_x
    	control_msg.linear.y = 0
    	control_msg.linear.z = 0
    	control_msg.angular.x = 0
    	control_msg.angular.y = 0
    	control_msg.angular.z = angular_z
        pub.publish(control_msg)

def get_target_angle(target_x, target_y):
    return math.atan2(target_y - y, target_x - x)

def get_distance(target_x, target_y):
    return math.sqrt(math.pow(target_y - y, 2) + math.pow(target_x - x, 2))

if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    target_x, target_y = center
    time.sleep(1)

    while not rospy.is_shutdown():
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        nonzero = img_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        if (len(nonzerox) > detect_threshold):
            position_x = np.mean(nonzerox)
            position_y = np.mean(nonzeroy)

            #print(position_x, position_y)

            if position_x < 210 and position_y < 160:
                target_x, target_y = top_left
            elif position_x < 210 and position_y < 320:
                target_x, target_y = middle_left
            elif position_x < 210 and position_y < 480:
                target_x, target_y = bottom_left
            elif position_x < 420 and position_y < 160:
                target_x, target_y = top_middle
            elif position_x < 420 and position_y < 320:
                target_x, target_y = center
            elif position_x < 420 and position_y < 480:
                target_x, target_y = bottom_middle
            elif position_x < 640 and position_y < 160:
                target_x, target_y = top_right
            elif position_x < 640 and position_y < 320:
                target_x, target_y = middle_top
            elif position_x < 640 and position_y < 480:
                target_x, target_y = bottom_right

        target_angle = get_target_angle(target_x, target_y)
        distance = get_distance(target_x, target_y)
        error = normalize_angle(target_angle - theta)

        # print("****************************")
        # print("x : ", x)
        # print("y : ", y)
        # print("theta : ", theta)
        # print("target_angle : ", target_angle)
        # print("error : ", error)
        # print("distance : ", distance)
        # print("****************************")

        if (distance < distacne_check):
            control_msg_publish(0, 0)
        else:
            if error > angle_check:
                control_msg_publish(3, 5)
            elif error < -angle_check:
                control_msg_publish(3, -5)
            else:
                control_msg_publish(3, 0)

        cv2.line(img, (0, 160), (639, 160), (0, 0, 0), 1)
        cv2.line(img, (0, 320), (639, 320), (0, 0, 0), 1)
        cv2.line(img, (210, 0), (210, 480), (0, 0, 0), 1)
        cv2.line(img, (420, 0), (420, 480), (0, 0, 0), 1)

        cv2.imshow("image", img)
        cv2.imshow("mask", img_mask)
        if cv2.waitKey(1) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
