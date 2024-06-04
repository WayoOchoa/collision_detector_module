#!/usr/bin/env python

import rospy
from collision_detection_module.srv import *

if __name__ == "__main__":
    rospy.init_node("service_caller",anonymous=True)
    rate = rospy.Rate(50)
    rospy.wait_for_service("/collision_detector/transfer_frame_data")
    rospy.sleep(5)
    while not rospy.is_shutdown():
        try:
            transfer_data = rospy.ServiceProxy("/collision_detector/transfer_frame_data",TransferData)
            resp = transfer_data(1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        rate.sleep()