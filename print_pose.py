#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def poseCallback(pose_message):
    rospy.loginfo("Pose x = %f", pose_message.x)
    rospy.loginfo("Pose y = %f", pose_message.y)
    rospy.loginfo("Pose theta = %f", pose_message.theta)
    

def main():
    rospy.init_node("print_pose_node")
    rospy.Subscriber("/turtle1/pose", Pose,  poseCallback)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=2)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Do something
        r.sleep()

    rospy.spin()  
    rospy.loginfo("Shutdown detected. Exiting");

if __name__ == '__main__':
    main()
