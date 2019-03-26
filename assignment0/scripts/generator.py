#!/usr/bin/env python

import rospy
from numpy.random import randint
from assignment0.msg import TwoInt

def generator(): 
    pub = rospy.Publisher('/numbers', TwoInt, queue_size = 10)
    rospy.init_node('generator', anonymous = True)
    Frequency = rospy.Rate(10)
    while not rospy.is_shutdown():
        num = TwoInt()
        num.num1 = randint(1, 100)
        num.num2 = randint(1, 100)
	rospy.loginfo(num)
        pub.publish(num)
        Frequency.sleep()

if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass
