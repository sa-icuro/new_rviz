#include "global.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include "ros/ros.h"

ros::NodeHandle mnh;
ros::Publisher my_pub;
geometry_msgs::PoseStamped goal;

ros::NodeHandle mnh2;
ros::Publisher my_pub2;
geometry_msgs::PoseStamped goal2;

void init_pub(){
   my_pub = mnh.advertise<geometry_msgs::PoseStamped>( "goalz", 1 );
}

void init_pub2(){
   my_pub2 = mnh2.advertise<geometry_msgs::PoseStamped>( "safepathz", 1 );
}

