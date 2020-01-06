#ifndef BASE_NODE_H
#define BASE_NODE_H

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <iostream>
#include "base_node.h"

namespace base_node {

template <typename ConfigT>
class BaseNodeDynamicReconfigure : public BaseNode {
 public:
  typedef typename dynamic_reconfigure::Server<ConfigT> DRServer;
  typedef typename DRServer::CallbackType DRSeverCallbackType;

  explicit BaseNodeDynamicReconfigure(ros::NodeHandle &nh,
                                      ros::NodeHandle &p_nh)
      : BaseNode(nh, p_nh), reconfigure_server_(p_nh) {
    initDynamicReconfigure();
  }

 protected:
  // callBackDynamicReconfigure(const ConfigT &config, uint32 level)
  virtual void callbackDynamicReconfigure(const ConfigT, uint32_t) {}

 private:
  DRServer reconfigure_server_;
  DRSeverCallbackType reconfigure_cb_;

  void initDynamicReconfigure() {
    reconfigure_cb_ = boost::bind(
        &BaseNodeDynamicReconfigure<ConfigT>::callbackDynamicReconfigure, this,
        _1, _2);
    reconfigure_server_.setCallback(reconfigure_cb_);
  }
};

}  // namespace base_node

#endif  // BASE_NODE_H
