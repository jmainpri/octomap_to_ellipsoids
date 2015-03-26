#!/usr/bin/python
#
#   Author: Jim Mainprice

import rospy
import math
from visualization_msgs.msg import *
from octomap_msgs.msg import *

from math import pi

from numpy import *

import time
import sys
import tf
import random

class ObjectStatePublisher:

    def __init__(self):

        self.object_pub = rospy.Publisher(rospy.get_namespace() + "visualization_marker_array", MarkerArray)

    def spawn_object1(self):
        #rospy.logdebug( "spawn objects" )
        
        listener = tf.TransformListener()
        rate = rospy.Rate(40.0)

        msg1 = Marker()
        msg1.header.frame_id = "world"
        msg1.header.stamp = rospy.Time.now()
        msg1.ns = "my_namespace"
        msg1.id = 1
        msg1.type = Marker.MESH_RESOURCE
        msg1.action = Marker.ADD
        msg1.lifetime = rospy.Time(0.3)
        
        msg1.color.a = 1.0
        msg1.color.r = 0.8
        msg1.color.g = 0.1
        msg1.color.b = 0.5
        msg1.mesh_use_embedded_materials = True
        msg1.mesh_resource = "package://touch_tomorrow_demo/models/helicopter.dae" 
      
        scale_factor_1 = 0.01#0.8

        msg2 = Marker()
        msg2.header.frame_id = "world"
        msg2.header.stamp = rospy.Time.now()
        msg2.ns = "my_namespace"
        msg2.id = 2
        msg2.type = Marker.MESH_RESOURCE
        msg2.action = Marker.ADD
        msg2.lifetime = rospy.Time(0.3)
        
        msg2.color.a = 1.0
        msg2.color.r = 0.9
        msg2.color.g = 0.01
        msg2.color.b = 0.05
        msg2.mesh_resource = "package://touch_tomorrow_demo/models/helicopter.dae"

        scale_factor_2 = 1.0

        msg3 = Marker()
        msg3.header.frame_id = "world"
        msg3.header.stamp = rospy.Time.now()
        msg3.ns = "my_namespace"
        msg3.id = 3
        msg3.type = Marker.MESH_RESOURCE
        msg3.action = Marker.ADD
        msg3.lifetime = rospy.Time(0.3)
        
        msg3.color.a = 1.0
        msg3.color.r = 0.8
        msg3.color.g = 0.7
        msg3.color.b = 0.3
        msg3.mesh_resource = "package://touch_tomorrow_demo/models/dragon_head.dae" 

        scale_factor_3 = 1.5

        object_1_in_scene = 0
        object_2_in_scene = 0
        object_3_in_scene = 0

        min_seen = 100

        while not rospy.is_shutdown():

            markerArray = MarkerArray()
            markerArray.markers = []

            try: # ELEPHANT
                now = rospy.Time.now()
                p1, q1 = listener.lookupTransform( '/world', "/vicon/TouchTomorrow1/TouchTomorrow1" , rospy.Time(0))
            except :
                object_1_in_scene = 0
            else:
                if object_1_in_scene <= min_seen : object_1_in_scene += 1
            #print "got transform3" 

            try: # CAMARO
                now = rospy.Time.now()
                p2, q2 = listener.lookupTransform( '/world', "/vicon/TouchTomorrow2/TouchTomorrow2" , rospy.Time(0))
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                object_2_in_scene = 0
            else:
                if object_2_in_scene <= min_seen : object_2_in_scene += 1

            try: # DRAGON
                now = rospy.Time.now()
                p3, q3 = listener.lookupTransform('/world', "/vicon/TouchTomorrow3/TouchTomorrow3", rospy.Time(0))
            except:
                object_3_in_scene = 0
            else:
                if object_3_in_scene <= min_seen : object_3_in_scene += 1

            print "objects ", object_1_in_scene, object_2_in_scene, object_3_in_scene

            #print p1
            # print q1

            if object_1_in_scene > min_seen : # ELEPHANT

                msg1.pose.position.x =  p1[0] #- 0.3 #+ offset_x -0.3 +
                msg1.pose.position.y =  p1[1] #- 11.3  #+ offset_y -11.3 +
                msg1.pose.position.z =  p1[2] #- 3.7 #+ offset_z -3.7 +
                #msg1.pose.position.x = p1[0] #+ offset_x
                #msg1.pose.position.y = p1[1] #+ offset_y
                #msg1.pose.position.z = p1[2] #+ offset_z
                msg1.pose.orientation.x = q1[0]
                msg1.pose.orientation.y = q1[1]
                msg1.pose.orientation.z = q1[2]
                msg1.pose.orientation.w = q1[3]
                msg1.scale.x = 1. * scale_factor_1
                msg1.scale.y = 1. * scale_factor_1
                msg1.scale.z = 1. * scale_factor_1

                # Offset in world
                pose = Pose()
                pose.position.x = -0.4 #-3.8 #+ offset_x -0.3 + #-0.4 (helicopter)
                pose.position.y = -0.6 #-2.  #+ offset_y -11.3 + #-0.6 (helicopter)
                pose.position.z = -0.45 #-1.5 #+ offset_z -3.7 + # -0.45 (helicopter)
                msg1.pose = ComposePoses( msg1.pose, pose )

                # self.object_pub.publish( msg1 )
                markerArray.markers.append( msg1 )

            if object_2_in_scene > min_seen : # CAMARO

                msg2.pose.position.x =  + p2[0] #+ offset_x
                msg2.pose.position.y =  + p2[1] #+ offset_y
                msg2.pose.position.z =  + p2[2] #+ offset_z
                msg2.pose.orientation.x = q2[0]
                msg2.pose.orientation.y = q2[1]
                msg2.pose.orientation.z = q2[2]
                msg2.pose.orientation.w = q2[3]
                msg2.scale.x = 1. * scale_factor_2
                msg2.scale.y = 1. * scale_factor_2
                msg2.scale.z = 1. * scale_factor_3

                pose = Pose()
                pose.position.x =  -1.8 #+ offset_x -0.3 +
                pose.position.y =  -1.9  #+ offset_y -11.3 +
                pose.position.z =  0.0 #+ offset_z -3.7 +
                #msg2.pose = ComposePoses( msg2.pose, pose )
                # Offset in object frame
                # mat = MakeTransformMatrix(rodrigues([pi/2, 0, 0]), transpose(matrix([0, 0, 0])))
                # msg1.pose = ComposePoses( PoseFromMatrix( mat ), msg1.pose )
                # self.object_pub.publish( msg2 )
                # markerArray.markers.append( msg2 )

            if object_3_in_scene > min_seen : # DRAGON

                msg3.pose.position.x =  + p3[0] #+ offset_x
                msg3.pose.position.y =  + p3[1] #+ offset_y
                msg3.pose.position.z =  + p3[2] #+ offset_z
                msg3.pose.orientation.x = q3[0]
                msg3.pose.orientation.y = q3[1]
                msg3.pose.orientation.z = q3[2]
                msg3.pose.orientation.w = q3[3]
                msg3.scale.x = 1. * scale_factor_3
                msg3.scale.y = 1. * scale_factor_3
                msg3.scale.z = 1. * scale_factor_3

                # Offset in world
                pose = Pose()
                pose.position.x =  -0.3 #+ offset_x -0.3 +
                pose.position.y =  -11.3  #+ offset_y -11.3 +
                pose.position.z =  -3.7 #+ offset_z -3.7 +
                msg3.pose = ComposePoses( msg3.pose, pose )

                # Offset in object frame
                # mat = MakeTransformMatrix(rodrigues([pi/2, 0, 0]), transpose(matrix([0, 0, 0])))
                # msg1.pose = ComposePoses( PoseFromMatrix( mat ), msg1.pose )

#                self.object_pub.publish( msg3 )
                markerArray.markers.append( msg3 )

            if len(markerArray.markers) > 0:
                self.object_pub.publish(markerArray)
                print "publish markers"
            rospy.sleep(0.01)


    def loop(self, hz=1.):
        r = rospy.Rate(hz) 
        while not rospy.is_shutdown():

            self.spawn_object1()
            #Spin
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('object_state_publisher',log_level=rospy.DEBUG)
    sp = ObjectStatePublisher()
    sp.loop(20.)

