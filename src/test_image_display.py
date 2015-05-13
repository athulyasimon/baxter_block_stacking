#!/usr/bin/python


import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import Image


def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def main():

	rospy.init_node('image_pusher')

	first_image = "/home/athulya/baxter_ws/src/baxter_block_stacking/src/block_stacking.png"

	send_image(first_image)



if __name__ == '__main__':
    main()
