#!/usr/bin/env python

from gridmapper.srv import *
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn.ensemble import AdaBoostClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.externals import joblib

bridge = CvBridge()
bdt = joblib.load('/home/shubh/catkin_ws/src/gridmapper/models/bdt.pkl')

def handle_classify(req):
    global bridge,bdt
    rospy.loginfo("Entered classifier")
    patch = bridge.imgmsg_to_cv2(req.image, "bgr8")
    # cv2.imshow("frame",patch)
    # cv2.waitKey(30)
    patch = patch.reshape(1,-1)
    Z = bdt.predict(patch)
    print Z
    return classifyResponse(Z)

def classify_server():
    rospy.init_node('classify_server')
    s = rospy.Service('classify', classify, handle_classify)
    print "Ready to classify patches."
    rospy.spin()

if __name__ == "__main__":
    classify_server()