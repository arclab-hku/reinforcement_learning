import rospy
import unittest
rospy.init_node("fucking test")
print(rospy.get_time())

rospy.sleep(1.0/100)
print(rospy.get_time())