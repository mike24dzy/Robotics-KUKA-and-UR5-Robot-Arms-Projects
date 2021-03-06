#!/usr/bin/env python 

import rospy 
import numpy as np
import tf 
import tf2_ros
import geometry_msgs.msg
import math as mt

def publish_transforms():
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0.0)
    T1 = np.dot(tf.transformations.quaternion_matrix(q1),                   
    tf.transformations.translation_matrix((1.5, 0.8, 0.0)))
    r1 = tf.transformations.quaternion_from_matrix(T1)
    tr1 = tf.transformations.translation_from_matrix(T1)
    t1.transform.rotation.x = r1[0]
    t1.transform.rotation.y = r1[1]
    t1.transform.rotation.z = r1[2]
    t1.transform.rotation.w = r1[3]
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]

    br.sendTransform(t1)

    
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
    T2 = np.dot(tf.transformations.quaternion_matrix(q2),
    tf.transformations.translation_matrix((0.0, 0.0, -2.0)))
    r2 = tf.transformations.quaternion_from_matrix(T2)
    tr2 = tf.transformations.translation_from_matrix(T2)
    t2.transform.rotation.x = r2[0]
    t2.transform.rotation.y = r2[1]
    t2.transform.rotation.z = r2[2]
    t2.transform.rotation.w = r2[3]
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]

    br.sendTransform(t2) 


    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"
    # Get the location of the origin of the camera_frame
    # Translation from robot frame to camera frame is (0.3, 0.0, 0.3)
    trans3 = np.array([0.3, 0.0, 0.3, 1])
    cloc = np.dot(T2, trans3) # the location of camera origin according to base
    vfromCtoO = np.array([tr1[0]-cloc[0], tr1[1]-cloc[1], tr1[2]-cloc[2]]) # vector 
    # -from camera to object
    # then set a unit vector with only x-axis
    unitv = np.array([1, 0, 0])
    # now if we want to have the angle between the vector and the x axis
    vx = np.dot(vfromCtoO, unitv)
    v = mt.sqrt(vfromCtoO[0]**2 + vfromCtoO[1]**2 + vfromCtoO[2]**2)
    angle = mt.acos(vx/v) # now we have the angle
    axis = np.cross(unitv, vfromCtoO) # rotation axis   
    q3r = tf.transformations.quaternion_about_axis(angle, (axis[0], axis[1], axis   
    [2]))
    q3matrix = tf.transformations.quaternion_matrix(q3r)
    qq2 = tf.transformations.quaternion_matrix(q2)
    q2inverse = tf.transformations.inverse_matrix(qq2) # get the inverse of the b to r rotation matrix
    q3 = np.dot(q2inverse, q3matrix)
    r3 = tf.transformations.quaternion_from_matrix(q3) # final rotation matrix
    # the translation is just trans3 and now we can start the transform
    t3.transform.translation.x = trans3[0]
    t3.transform.translation.y = trans3[1]
    t3.transform.translation.z = trans3[2]
    t3.transform.rotation.x = r3[0]
    t3.transform.rotation.y = r3[1]
    t3.transform.rotation.z = r3[2]    
    t3.transform.rotation.w = r3[3]
    
    br.sendTransform(t3)

if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.15)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.15)

    
