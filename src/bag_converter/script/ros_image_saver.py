#!/usr/bin/python2
# -*- coding: utf-8 -*-
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import argparse
import os

'''
Argparser
'''
parser = argparse.ArgumentParser()
parser.add_argument(
        "--topic",
        help="specify ros topic", 
        type=str,
        required=True,
        )
parser.add_argument(
        "--save_dir",
        help="specify save dir", 
        type=str,
        required=True,
        )
args = parser.parse_args()

'''
Instance a cv_bridge
'''
CV_BRIDGE = CvBridge()

class ImageSubscriber(object):
    def __init__(self, topic="", save_dir=""):
        self.topic = topic
        self.save_dir = save_dir
        rospy.Subscriber(topic, Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Received image and Saving ...')
        try:
            cv2_img = CV_BRIDGE.imgmsg_to_cv2(msg, "bgr8")
        
        except CvBridgeError, e:
            rospy.logerr(e)
        
        else:
            frame_id = msg.header.frame_id
            image_dir = os.path.join(self.save_dir, frame_id)
            if not os.path.exists(image_dir):
                os.makedirs(image_dir)
            image_name = str(msg.header.stamp.secs) + '.' + str(msg.header.stamp.nsecs) + '.png'
            image_path = os.path.join(image_dir, image_name)
            cv2.imwrite(image_path, cv2_img)

def init():
    '''
    init node
    '''
    rospy.init_node('rosbag_2_image_listener')
    
    '''
    Subscriber
    '''
    target_topic = args.topic
    target_save_dir = args.save_dir
    sub = ImageSubscriber(
            topic=target_topic, 
            save_dir=target_save_dir
            )
    
    '''
    Looping
    '''
    rospy.spin()

if __name__ == '__main__':
    init()
