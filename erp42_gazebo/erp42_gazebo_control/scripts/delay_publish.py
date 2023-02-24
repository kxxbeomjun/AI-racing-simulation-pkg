#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist
    
def get_cmd(msg):
    #rospy.loginfo("Get linear: {0}".format(msg.linear.x))
    cmd_vel_Q.append(msg)
    pub_cmd()

def pub_cmd():
    if len(cmd_vel_Q) > (int)(delayTime * controller_freq):
        delayed_vel = cmd_vel_Q.pop(0)
        # rospy.loginfo("Check delayed linear: {0}".format(delayed_vel.linear.x))
        cmd_vel_pub.publish(delayed_vel)
    else:
        rospy.loginfo("wait publish")

rospy.init_node('delay_publish', anonymous=True)
delayTime = 0.05*6
cmd_vel_Q = []

# cmd_vel_delay publisher
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

try:
    isController = rospy.search_param('move_base/controller_frequency')
    while isController == None:
        print("Controller has not been launched!")
        rospy.sleep(0.5)

    controller_freq = rospy.get_param(isController)
    # cmd_vel subscriber
    rospy.Subscriber("cmd_vel_real", Twist, get_cmd, queue_size=1)
    rospy.spin()
except rospy.ROSInternalException:
    pass
  