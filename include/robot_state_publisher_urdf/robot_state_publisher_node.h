#ifndef ROBOT_STATE_PUBLISHER_NODE_H
#define ROBOT_STATE_PUBLISHER_NODE_H

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "robot_state_publisher_urdf/robot_state_publisher_urdf.h"

using namespace std;
using namespace ros;
using namespace KDL;

typedef std::map<std::string, urdf::JointMimicSharedPtr > MimicMap;


#endif
