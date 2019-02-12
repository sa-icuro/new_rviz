#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import numpy
import random

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import sys, termios, tty, os, time

import rviz_tools_py as rviz_tools
global X 
markers = rviz_tools.RvizMarkers('/map', 'visualization_marker')
path_route = []
path_populate = []
path_safe = []
global ONCE
# Define exit handler
def cleanup_node():
    print "Shutting down node"
    #markers.deleteAllMarkers()

#rospy.on_shutdown(cleanup_node)


def callback(msg):
    rospy.loginfo("Received a Safepoint!")
    #rospy.loginfo("Timestamp: " + str(msg.header.stamp))
    #rospy.loginfo("frame_id: " + str(msg.header.frame_id))

    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    rospy.loginfo("Safepoint data: [ %f, %f, %f ]"%(position.x, position.y, quat.z))

    # Publish a sphere using a ROS Pose
    P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
    cube_width = 0.2 # diameter
    color = 'orange'
    markers.publishCube(P, color, cube_width, 0) # pose, color, scale, lifetime
    #markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime

    #cube_width = 0.6
    #markers.publishCube(P, 'blue', cube_width, 5.0)

    #Write to Goals.yaml as used by the magni_goal_sender package
    f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/safe_waypoints.txt","a+")
    f.write("- %f, %f, %f\n" % (position.x, position.y, quat.z))
    f.close()

def callback2(msg):
    rospy.loginfo("Received a Goal!")

    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    rospy.loginfo("Goal data (Appending to route): [ %f, %f, %f ]"%(position.x, position.y, quat.z))

    # Publish a sphere using a ROS Pose
    P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
    scale = Vector3(0.3,0.3,0.3) # diameter
    color = 'blue'
    markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime

    path_route.append( Point(position.x,position.y,0) )
    width = 0.055
    markers.publishPath(path_route, 'white', width, 0) # path, color, width, lifetime    

    #Write to Goals.yaml as used by the magni_goal_sender package
    #x = int(position.x), y = int(position.y), z = int(quat.z)
    #open('/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals_0.yaml', 'w').close()
    #if (count == 0):
        #open('/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals_0.yaml', 'w').close()    
    f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml","a+")
    #f.write("goals: \n")
    #count += 1

    f.write("- %f, %f, %f, 0\n" % (position.x, position.y, quat.z))
    f.close()

def callback3(msg):

    rospy.loginfo("Received a Path!")

    # Copying for simplicity
    position = msg.pose.position
    quat = msg.pose.orientation
    rospy.loginfo("Path data (Appending to SafePath): [ %f, %f, %f ]"%(position.x, position.y, quat.z))

    # Publish a sphere using a ROS Pose
    P = Pose(Point(position.x,position.y,0),Quaternion(0,0,quat.z,1))
    scale = Vector3(0.275,0.275,0.275) # diameter
    color = 'purple'
    markers.publishSphere(P, color, scale, 0) # pose, color, scale, lifetime

    path_safe.append( Point(position.x,position.y,0) )
    width = 0.02
    markers.publishPath(path_safe, 'green', width, 0) # path, color, width, lifetime    

    f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/safe_paths.txt","a+")
    f.write("%f, %f, %f\n" % (position.x, position.y, quat.z))
    f.close()

def listener():
    rospy.init_node('listener', anonymous=True)
    populate()
    rospy.Subscriber("/safepointz", PoseStamped, callback)
    rospy.Subscriber("/goalz", PoseStamped, callback2)
    rospy.Subscriber("/safepathz", PoseStamped, callback3)

    rospy.spin()

def populate():

    f = open("/home/centaur/catkin_ws/src/magni_goal_sender/param/safe_paths.txt","a+")
    first_val = 0
    file_content = f.readlines()
    for lines in file_content:
        data = lines.split(",")
        x = float(data[0].strip())
        y = float(data[1].strip())
        z = float(data[2].strip())

        P = Pose(Point(x,y,0),Quaternion(0,0,z,1))
        scale = Vector3(0.275,0.275,0.275) # diameter
        scale_first = Vector3(0.43,0.43,0.43) # diameter
        if first_val==0:
            rospy.sleep(0.2)
            markers.publishCube(P, 'white', 0.3, 0) # pose, color, scale, lifetime
            #print "published cube"
            first_val = 1
            path_populate.append( Point(x,y,0) )
            width = 0.02
            markers.publishPath(path_populate, 'green', width, 0) # path, color, width, lifetime
            #print "Done with first_val"
        else:
            #print "first_val = 1"
            #print P
            markers.publishSphere(P, 'purple', scale, 0) # pose, color, scale, lifetime
            rospy.sleep(0.015)
            markers.publishCube(P, 'orange', 0.2, 0) # pose, color, scale, lifetime
            rospy.sleep(0.015)
            path_populate.append( Point(x,y,0) )
            width = 0.02
            markers.publishPath(path_populate, 'green', width, 0) # path, color, width, lifetime   
    f.close()

if __name__ == '__main__':
    listener()