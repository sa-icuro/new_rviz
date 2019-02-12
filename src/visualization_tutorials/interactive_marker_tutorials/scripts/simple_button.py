#!/usr/bin/env python
import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

pub = rospy.Publisher('pose_out', PoseStamped, queue_size=10)

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "map"
        pose.pose = feedback.pose
        pub.publish(pose)

def makeMarker():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.g = 1.0
    marker.color.a = 1.0
    return marker

def makeButton():
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.g = 1.0
    marker.color.a = 1.0
    return marker

if __name__=="__main__":
    rospy.init_node("pose_marker_server")

    # create an interactive marker server on the topic namespace goal_marker
    server = InteractiveMarkerServer("int_marker")

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.scale = 0.7

    # create marker and a non-interactive control containing the marker
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeMarker())

    # add control to interactive marker
    int_marker.controls.append(control)

    # add control for position (as it has no marker, RViz decides which one to use).
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0    

    # add control for pose publishing.
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.markers.append(makeButton()) # This is required, it will not work otherwise.
    int_marker.controls.append(control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()