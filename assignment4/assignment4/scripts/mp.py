#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    def generate_randsample(self):
	qrand = []
	if self.num_joints == 6:	
	    for i in range(self.num_joints):
	        rand = random.uniform(-2*numpy.pi, 2*numpy.pi)
	        qrand.append(rand)
	    return qrand
	elif self.num_joints == 7:
	    for i in range(self.num_joints):
		rand = random.uniform(-numpy.pi, numpy.pi)
		qrand.append(rand)
	    return qrand
	else:
	    return []

    def distance_calc(self, q1, q2):
	q = numpy.subtract(q1,q2)
	d = numpy.linalg.norm(q) 
	return d

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
	print(self.q_current)
	translation = tf.transformations.translation_matrix((ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z))
	rotation = tf.transformations.quaternion_matrix((ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w))
	T_goal = numpy.dot(translation, rotation)
	
	q_goal = numpy.array(self.IK(T_goal)) # goal
	q_curr = numpy.array(self.q_current)  # start 
	self.parents = []
	self.all_q = [q_curr]
	self.path = []
	self.go = []
	self.shortcut = []

	while True: 
	    q_new = self.generate_randsample()
	    while self.is_state_valid(q_new) == False:
	        q_new = self.generate_randsample()
	
	    print("qnew",q_new)
	    # find the minimum distance q and the parent and then continue below
	    
	    if len(self.all_q) == 1:
	        q_parent = self.all_q[0]
	    else:
		dis_comp = []
	        for i in range(len(self.all_q)):
		    d = self.distance_calc(q_new, self.all_q[i])
		    dis_comp.append(d)
	        mini = min(dis_comp)
	        min_index = dis_comp.index(mini)
	        q_parent = self.all_q[min_index]
	    #print("qparent",q_parent)
	    # need to check for obstacles here
	    uni_vec = numpy.subtract(q_new,q_parent)/numpy.linalg.norm(numpy.subtract(q_new,q_parent))
	    di = self.distance_calc(q_new, q_parent)
	    k = di/40
	    qnew_branch = q_parent
	    print('qnew_branch',qnew_branch)

	    for i in range(40):
		qnew_branch = qnew_branch + k*uni_vec
		if self.is_state_valid(qnew_branch) == False:
		    qnew_branch -= k*uni_vec
		    break
	    #if self.is_state_valid(qnew_branch) == False:
		#continue
	    #while self.is_state_valid(qnew_branch) == True:
		#qnew_branch += k*uni_vec
	    #qnew_branch = qnew_branch - k*uni_vec
	
	    # trace branch and save parents below
	    self.all_q.append(qnew_branch)
	    y = RRTBranch(q_parent, qnew_branch)
	    self.parents.append(y)
	    # self.parents.append(q_parent)
	    
	    

	    # check if the qnew_branch can connect to the goal

	    q_to_goal = qnew_branch
	    dis_goal = self.distance_calc(q_goal, q_to_goal)
	    print("disgoal",dis_goal)
	    g = dis_goal/40
	    uni_vec_goal = numpy.subtract(q_goal,q_to_goal)/numpy.linalg.norm(numpy.subtract(q_goal,q_to_goal))
	    print("uni2", uni_vec_goal)
	    

	    for i in range(40):
		q_to_goal = q_to_goal + g*uni_vec_goal
		if self.is_state_valid(q_to_goal) == False:
		    print("no")
		    break
	    	elif numpy.linalg.norm(numpy.subtract(q_goal,q_to_goal)) < 0.0000001:
		    print("yes")	    	
		    self.all_q.append(q_goal)
	    	    x = RRTBranch(qnew_branch, q_goal)
	    	    self.parents.append(x)
	    	
			
	    #while self.is_state_valid(q_to_goal) == True:
		#q_to_goal = q_to_goal + g*uni_vec_goal
		#if self.is_state_valid(q_to_goal) == False: 
		    #print("no")
		    #q_to_goal -= g*uni_vec_goal
		    #break
		#elif numpy.linalg.norm(numpy.subtract(q_goal, q_to_goal)) < 0.001:
		    #print("yes")
		    #self.all_q.append(q_goal)
		    #x = RRTBranch(qnew_branch, q_goal)
		    #self.parents.append(x)
		    #self.parents.append(qnew_branch)
		    #self.parents.append(q_goal)
		    #break
	    
	    #print("check",self.all_q[-1])
	    #print("q_goal",q_goal)
	    print('----------------------')
	    if numpy.linalg.norm(numpy.subtract(q_goal,self.all_q[-1])) == 0: 
		break
	#print("branch",len(self.parents))
	
######## FIND ACTUAL PATH

	self.path.append(q_goal)
	self.path.append(self.parents[-1].parent)
	print(self.path)
	h = -1
	flag = True
	while True:
	    for i in range(len(self.parents)-1):
	    	if numpy.linalg.norm(numpy.subtract(self.parents[i].q,self.parents[h].parent))<0.000001:
	    	    self.path.append(self.parents[i].parent)
	    	    h = -(len(self.parents)-i)
	    	    print('h', h)
	    for k in range(len(self.path)):
	    	if numpy.linalg.norm(numpy.subtract(self.path[k],q_curr))<0.000001:
	    	   flag = False
	    if flag != True:
	    	break
	self.path.append(q_curr)
	    #print(i)
	    #print(len(self.parents))
	    #print(self.parents)
	    #self.path.append(self.parents[i].parent)
	    #if self.parents[i].parent = q_curr:
	    #	self.path.append(q_curr)
		#break
	    #else:
		#if 
		#i -= 1
	#print("path",len(self.path))
	self.path.reverse()
	print('path',self.path)
	print("line-----------------------------------line")

	'''self.shortcut.append(self.path[0])
	print("shortcut start", self.shortcut)
	a = 0

	for z in range(len(self.path)-2):
	    qs = self.path[a]
	    qtest = qs
	    if len(self.path) <= 3:
	    	break 
	    else:
		norm = numpy.linalg.norm(numpy.subtract(self.path[z+1],qs))
		if norm == 0: 
		    continue
		else:
		    u = numpy.subtract(self.path[z+1],qs)/norm
		    ds = self.distance_calc(self.path[z+1],qs)
		    s = ds/50
		    for i in range(50):
			qtest += s*u
			if self.is_state_valid(qtest) == True:
			    continue
			elif self.is_state_valid(qtest) == False: 
			    self.shortcut.append(self.path[z])
			    print('APPEND',self.path[z])
			    a = z
			    break
	print("path after",self.path)
	self.shortcut.append(q_goal)
	print('shortcut',self.shortcut)'''

	self.go.append(self.path[0])
	for i in range(len(self.path)-1):
	    if numpy.linalg.norm(numpy.subtract(self.path[i+1],self.path[i])) == 0:
	        continue
	    else: 
	        dq = self.distance_calc(self.path[i+1], self.path[i])
	        uni = numpy.subtract(self.path[i+1],self.path[i])/numpy.linalg.norm(numpy.subtract(self.path[i+1],self.path[i]))
		#print('UNI!!!',uni)
	        seg = dq/40
	        #print('SEG',seg)
	        q_s = self.path[i] + seg*uni
	        self.go.append(q_s)
	        for i in range(39):
		        q_s = q_s + seg*uni
		        self.go.append(q_s)
	self.go.pop()
	self.go.append(self.path[-1])
	#print(self.go)   
	

	########### SHORTCUTTING !!! ############
		
	    


	#print("final path", self.go)
	j = JointTrajectory()
	j.joint_names = self.joint_names
	qq = []
	for i in range(numpy.size(self.go)/self.num_joints):
	    f = JointTrajectoryPoint()
	    f.positions = self.go[i]
	    qq.append(f)
	j.points = qq
	self.pub.publish(j)    



        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

