#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

extern ros::NodeHandle mnh;
extern void init_pub();
extern ros::Publisher my_pub;
extern geometry_msgs::PoseStamped goal;

//if 0, then path
//if 1, then route
extern int startup;

extern ros::NodeHandle mnh2;
extern void init_pub2();
extern ros::Publisher my_pub2;
extern geometry_msgs::PoseStamped goal2;
