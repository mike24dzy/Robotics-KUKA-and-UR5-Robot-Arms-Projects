#!/usr/bin/env python 

import rospy
from std_msgs.msg import Int16
from assignment0.msg import TwoInt
import numpy as np

def callback(data):
    sixt = Int16()
    sixt.data = data.num1 + data.num2
    rospy.loginfo(sixt)
    pub = rospy.Publisher('/sum', Int16, queue_size = 10)
    pub.publish(sixt)

def adder():
    num = TwoInt()
    rospy.init_node('adder', anonymous = True)
    rospy.Subscriber("/numbers", TwoInt, callback)

    rospy.spin()

if __name__ == '__main__':
    adder()

    

