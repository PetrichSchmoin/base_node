#ifndef BASE_NODE_H
#define BASE_NODE_H

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace base_node {

class BaseNode {
 public:
  explicit BaseNode(ros::NodeHandle &nh, ros::NodeHandle &p_nh)
      : nh_(nh), p_nh_(p_nh) {}

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle p_nh_;
};

}  // namespace base_node

#endif  // BASE_NODE_H
