#!/usr/bin/env python

from gridmapper.srv import *
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import oll
import math

def sigmoid(x):
  return 1 / (1 + math.exp(-x))

bridge = CvBridge()
o = oll.oll("CW", C=1.0, bias=0.0)
o.load('/home/shubh/catkin_ws/src/gridmapper/models/oll.model')
def handle_classify(req):
    global bridge,bdt
    rospy.loginfo("Entered classifier")
    patch = bridge.imgmsg_to_cv2(req.image, "bgr8")
    # cv2.imshow("frame",patch)
    # cv2.waitKey(30)
    patch = patch.reshape(1,-1)
    if req.flag!=0:
        labls = [req.flag]
        o.fit(patch,np.array(labls))
    probl = o.decision_function(patch)
    Z = sigmoid(probl[0]/100.0)
    return classifyResponse(Z)

def classify_server():
    rospy.init_node('classify_server')
    s1 = rospy.Service('classify', classify, handle_classify)
    print "Ready to classify patches."
    rospy.spin()

if __name__ == "__main__":
    classify_server()
