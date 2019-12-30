#ifndef BASE_NODE_H
#define BASE_NODE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "./base_node.h"
#include <iostream>

namespace base_node {

    class BaseNode {
      public:
        explicit BaseNode(ros::NodeHandle &nh, ros::NodeHandle &p_nh):
          nh_(nh), p_nh_(p_nh)
        {}

      protected:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
    };

  template <typename ConfigT>
  class BaseNodeDynamicReconfigure : public BaseNode {
  public:
    typedef typename dynamic_reconfigure::Server<ConfigT> DRServer;
    typedef typename DRServer::CallbackType DRSeverCallbackType;

    explicit BaseNodeDynamicReconfigure(ros::NodeHandle &nh, ros::NodeHandle &p_nh):
      BaseNode(nh, p_nh),
      reconfigure_server_(p_nh)
    {
      initDynamicReconfigure();
    }

  protected:
    virtual void callbackDynamicReconfigure(const ConfigT &config, uint32_t level) {}

  private:
    DRServer reconfigure_server_;
    DRSeverCallbackType reconfigure_cb_;

    void initDynamicReconfigure()
    {
      reconfigure_cb_ = boost::bind(&BaseNodeDynamicReconfigure<ConfigT>::callbackDynamicReconfigure, this, _1, _2);
      reconfigure_server_.setCallback(reconfigure_cb_);
    }

  };

}

#endif // BASE_NODE_H
