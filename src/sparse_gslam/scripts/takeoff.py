#!/usr/bin/env python
from std_msgs.msg import String
from std_srvs.srv import SetBool
import rospy

if __name__ == "__main__":
    rospy.init_node("takeoff_commander")
    rospy.wait_for_service('takeoff')
    takeoff = rospy.ServiceProxy('takeoff', SetBool)

    takeoff(True)

    rospy.spin()

    takeoff(False)