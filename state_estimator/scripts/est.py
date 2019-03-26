#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####
	## Prediction 
	
	xhat = numpy.zeros((3,1))
	
	xhat[0][0] = self.x[0][0] + self.step_size*sens.vel_trans*math.cos(self.x[2][0])
	xhat[1][0] = self.x[1][0] + self.step_size*sens.vel_trans*math.sin(self.x[2][0])
	xhat[2][0] = self.x[2][0] + self.step_size*sens.vel_ang

	#self.x = xhat 

	## After getting the xhat so we can differentiate f to obtain F 

	F = numpy.zeros((3,3))

	F[0][0] = 1
	F[1][0] = 0
	F[2][0] = 0
	F[0][1] = 0
	F[1][1] = 1
	F[2][1] = 0
	F[0][2] = -self.step_size*sens.vel_trans*math.sin(self.x[2][0])
	F[1][2] = self.step_size*sens.vel_trans*math.cos(self.x[2][0])
	F[2][2] = 1

	## Predict Covariance Phat

	Phat = self.V + numpy.dot(numpy.dot(F,self.P),numpy.transpose(F))

	H = 0 
	S = 0 
	R = 0
	nu = 0
	W = 0 
	y_kplus1 = 0 
	h = 0 
	
	## Calculating Matrice
	if len(sens.readings) > 0:
		H = numpy.zeros((2*len(sens.readings),3))
		for i in range(len(sens.readings)):
			if math.sqrt((xhat[0][0]-sens.readings[i].landmark.x)**2+(xhat[1][0]-sens.readings[i].landmark.y)**2) > 0.1:
				H[2*i][0] = (xhat[0][0]-sens.readings[i].landmark.x)/(math.sqrt((xhat[0][0]-sens.readings[i].landmark.x)**2+(xhat[1][0]-sens.readings[i].landmark.y)**2))
				H[2*i+1][0] = -(xhat[1][0]-sens.readings[i].landmark.y)/((xhat[0][0]-sens.readings[i].landmark.x)**2+(xhat[1][0]-sens.readings[i].landmark.y)**2)
				H[2*i][1] = (xhat[1][0]-sens.readings[i].landmark.y)/(math.sqrt((xhat[0][0]-sens.readings[i].landmark.x)**2+(xhat[1][0]-sens.readings[i].landmark.y)**2))
				H[2*i+1][1] = (xhat[0][0]-sens.readings[i].landmark.x)/((xhat[0][0]-sens.readings[i].landmark.x)**2+(xhat[1][0]-sens.readings[i].landmark.y)**2)
				H[2*i][2] = 0
				H[2*i+1][2] = -1
		
		W = numpy.zeros((2*len(sens.readings),2*len(sens.readings)))
		for j in range(len(sens.readings)):
			W[2*j][2*j] = 0.1
			W[2*j+1][2*j+1] = 0.05

		y_kplus1 = numpy.zeros((2*len(sens.readings),1))
		for z in range(len(sens.readings)):
			y_kplus1[2*z][0] = sens.readings[z].range
			y_kplus1[2*z+1][0] = sens.readings[z].bearing

		h = numpy.zeros((2*len(sens.readings),1))
		for v in range(len(sens.readings)):
			h[2*v][0] = math.sqrt((xhat[0][0]-sens.readings[v].landmark.x)**2+(xhat[1][0]-sens.readings[v].landmark.y)**2)
			h[2*v+1][0] = math.atan2(sens.readings[v].landmark.y-xhat[1][0],sens.readings[v].landmark.x-xhat[0][0])-xhat[2][0]

		nu = y_kplus1-h
		for b in range(len(sens.readings)):
			if nu[2*b+1] < -numpy.pi:
				nu[2*b+1][0] += 2*numpy.pi 
			elif nu[2*b+1] > numpy.pi:
				nu[2*b+1][0] -= 2*numpy.pi
		S = W + numpy.dot(numpy.dot(H,Phat),numpy.transpose(H))
		R = numpy.dot(numpy.dot(Phat,numpy.transpose(H)),numpy.linalg.inv(S))

	## output x and P

	self.x = xhat + numpy.dot(R,nu)
	self.P = Phat - numpy.dot(numpy.dot(R,H),Phat)

        #### ----- YOUR CODE GOES HERE ----- ####
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
