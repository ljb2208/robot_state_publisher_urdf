#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher_urdf/robot_state_publisher_urdf.h"
#include "robot_state_publisher_urdf/robot_state_publisher_node.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace robot_state_publisher_urdf;

// ----------------------------------
// ----- MAIN -----------------------
// ----------------------------------
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "robot_state_publisher_urdf");
  NodeHandle node;

  // gets the location of the robot description on the parameter server
  urdf::Model model;
  if (!model.initParam("robot_description"))
    return -1;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  int publish_freq;

  node.param<int>("publish_freq", publish_freq, 1);

  MimicMap mimic;

  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++) {
    if(i->second->mimic) {
      mimic.insert(make_pair(i->first, i->second->mimic));
    }
  }

  RobotStatePublisherURDF state_publisher(tree, model);
  //JointStateListener state_publisher(tree, mimic, model);
  ros::Rate r(publish_freq);
  while (ros::ok())
  {     
    state_publisher.publishFixedTransforms("", false);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}