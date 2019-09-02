#!/usr/bin/env python
import rospy
import geometry_msgs
import turtlesim


def poseCallback(pose_message):
    rospy.loginfo("Pose x = %f", pose_message.x)
    rospy.loginfo("Pose y = %f", pose_message.y)
    rospy.loginfo("Pose theta = %f", pose_message.theta)
    

def main():

  rospy.init_node("print_pose_node")
  rospy.Subscriber("/turtle1/pose", turtlesim.msg.Pose,  poseCallback)
  pub = rospy.Publisher("/turtle1/cmd_vel", geometry_msgs.msg.Twist, queue_size=2)

  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
      # Do something
      rospy.spinOnce()
      r.sleep()

  rospy.loginfo("Shutdown detected. Exiting");

if __name__ == '__main__':
    main()
