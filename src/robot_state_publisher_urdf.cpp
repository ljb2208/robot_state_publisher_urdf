#include <kdl/frames_io.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher_urdf/robot_state_publisher_urdf.h"

using namespace std;
using namespace ros;

namespace robot_state_publisher_urdf {
RobotStatePublisherURDF::RobotStatePublisherURDF(const KDL::Tree& tree, const urdf::Model& model)
  : model_(model)
{  
  
  // walk the tree and add segments to segments_
  addChildren(tree.getRootSegment());


}

// add children to correct maps
void RobotStatePublisherURDF::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (unsigned int i=0; i<children.size(); i++) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model_.getJoint(child.getJoint().getName()) && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
        ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", root.c_str(), child.getName().c_str());
      }
      else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
      }
    }
    else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}


// publish moving transforms
void RobotStatePublisherURDF::publishTransforms(const map<string, double>& joint_positions, const Time& time, const std::string& tf_prefix)
{
  ROS_DEBUG("Publishing transforms for moving joints");
  std::vector<geometry_msgs::TransformStamped> tf_transforms;

  // loop over all joints
  for (map<string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++) {
    std::map<std::string, SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(jnt->second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
      tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
      tf_transforms.push_back(tf_transform);
    }
    else {
      ROS_WARN_THROTTLE(10, "Joint state with name: \"%s\" was received but not found in URDF", jnt->first.c_str());
    }
  }
  tf_broadcaster_.sendTransform(tf_transforms);
}

// publish fixed transforms
void RobotStatePublisherURDF::publishFixedTransforms(const std::string& tf_prefix, bool use_tf_static)
{
  ROS_DEBUG("Publishing transforms for fixed joints");
  std::vector<geometry_msgs::TransformStamped> tf_transforms;
  geometry_msgs::TransformStamped tf_transform;

  // loop over all fixed segments
  for (map<string, SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++) {
    geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
    tf_transform.header.stamp = ros::Time::now();
    if (!use_tf_static) {
      tf_transform.header.stamp += ros::Duration(0.5);
    }
    tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
    tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
    tf_transforms.push_back(tf_transform);
  }
  if (use_tf_static) {
    static_tf_broadcaster_.sendTransform(tf_transforms);
  }
  else {
    tf_broadcaster_.sendTransform(tf_transforms);
  }
}

}