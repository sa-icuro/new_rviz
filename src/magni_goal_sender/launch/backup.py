'''
This code will run at the startup of the magni and achieve the following outcomes:
-> Read from safepoints and paths files and populate list_points[] and paths{}
-> There will also be a callback listening for a 'param request' from the client
	-> Upon catching the request, the data strucures will sent as a a string response
	   which is then converted back into the appropriate data structure formate on the 
	   client's side allowing the data to be stored on the magni instead of locally. 
'''

import random
import re

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()
import sys, termios, tty, os, time
import rviz_tools_py as rviz_tools
import json


##### FUNCTIONS TO LOAD FROM FILE #####

def byteify(input):
	if isinstance(input, dict):
		return {byteify(key): byteify(value)
				for key, value in input.iteritems()}
	elif isinstance(input, list):
		return [byteify(element) for element in input]
	elif isinstance(input, unicode):
		return input.encode('utf-8')
	else:
		return input

def isclose(a, b, rel_tol=1e-05, abs_tol=0.0):
	return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

##### END OF FUNCTIONS TO LOAD FROM FILE #####

#Load Points
list_points = []
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt')) > 0:
	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/points.txt', 'rb') as f:
		for x in f.readlines():
			temp_tup1 = x.replace("(","")
			temp_tup2 = temp_tup1.replace(" ","")
			temp_tup1 = temp_tup2.replace(")","")
			temp_tup2 = temp_tup1.split(",")
			list_points.append((int(temp_tup2[0]), float(temp_tup2[1]), float(temp_tup2[2]), float(temp_tup2[3])))
print "\nLIST_POINTS: " + str(list_points)
#list_paths = []


#Load Paths
paths = dict((k,[]) for k in range(len(list_points)))
if(os.path.getsize('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json')) > 0:
	with open('/home/centaur/catkin_ws/src/magni_goal_sender/param/paths.json', "r") as f:
		paths = byteify(json.load(f))
print "\nPATHS: " + str(paths)

def callback_respond(data):
	#rospy.sleep(0.5)
	paths2 = {}
	#paths3 = {}
	print "\npaths before: " + str(paths) + "\n"
	msg = '|'.join(str(key)+ ":" + str(val) for (key,val) in paths.iteritems())
	print msg
	'''
	data = msg.split('|')
	for item in data:
		val_id = int(item[0])
		paths2[str(val_id)] = []
		#paths3[str(val_id+1)] = []
		tmp = item.split(':')
		tmp = str(tmp[1]).strip('[')
		tmp = tmp.strip(']')
		tmp = tmp.split(",")
		for x in tmp:
			x = str(x).strip(" ")
			paths2[str(val_id)].append(int(x))
			#paths3[str(val_id+1)].append(1+int(x))
	msg =  '\npaths after: ' + str(paths2) + '\n'
	print msg
	'''
	pub = rospy.Publisher('paramz_response', String, queue_size = 1)
	rospy.sleep(0.5)
	pub.publish(msg)
	rospy.sleep(0.5)

	'''Testing if the user creates new waypoints before clicking the load button
	dall = {}
	for d in [paths2, paths3]:
  		dall.update(d)

	print "\ncombined paths: " + str(dall) + "\n"
	'''

def callback_populate(data):
	msg = str(data.data)
	paths2 = {}
	data = msg.split('|')
	print "populate: " + str(data) + "\n"
	for item in data:
		#print "item"  + str(item) + "\n"
		#paths3[str(val_id+1)] = []
		tmp = item.split(':')
		val_id = int(tmp[0])
		paths2[str(val_id)] = []
		#print "tmp[1]: " + tmp[1] + "\n"
		tmp = str(tmp[1]).strip('[')
		tmp = tmp.strip(']')
		tmp = tmp.split(",")
		for x in tmp:
			x = str(x).strip(" ")
			print "\nx: " + str(x) + "\n" 
			try:
				print "Node: " + str(x) + " connected to node " + str(val_id) + "\n"
				paths2[str(val_id)].append(int(x))
			except:
				print "issue with adding Node " + str(x) + " node to " + str(val_id)
				#To catch error thrown by Null values
			#paths3[str(val_id+1)].append(1+int(x))

	msg =  '\npaths after: ' + str(paths2) + '\n'
	print msg
	populate(paths2)
	paths = paths2
	globals()['vertex_count'] = len(paths)
	print "done-we should end function"
