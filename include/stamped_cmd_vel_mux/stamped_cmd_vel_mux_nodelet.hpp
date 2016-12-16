/**
 * @file /include/stamped_cmd_vel_mux/stamped_cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the stamped_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef STAMPED_CMD_VEL_MUX_HPP_
#define STAMPED_CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include "stamped_cmd_vel_mux/reloadConfig.h"
#include "stamped_cmd_vel_mux/stamped_cmd_vel_subscribers.hpp"
#include "stamped_cmd_vel_mux/stamped_cmd_vel_publishers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class StampedCmdVelMuxNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

  StampedCmdVelMuxNodelet()
  {
    cmd_vel_subs.allowed = VACANT;
    dynamic_reconfigure_server = NULL;
  }

  ~StampedCmdVelMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

private:
  static const unsigned int VACANT       = 666666;  /**< ID for "nobody" active input; anything big is ok */
  static const unsigned int GLOBAL_TIMER = 888888;  /**< ID for the global timer functor; anything big is ok */

  StampedCmdVelSubscribers cmd_vel_subs;  /**< Pool of cmd_vel topics subscribers */
  StampedCmdVelPublishers  cmd_vel_pubs;  /**< Multiplexed command velocity topic */
  ros::Publisher active_subscriber;       /**< Currently allowed cmd_vel subscriber */
  ros::Timer common_timer;                /**< No messages from any subscriber timeout */
  double common_timer_period;             /**< No messages from any subscriber timeout period */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, unsigned int idx);
  void stampedCmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<stamped_cmd_vel_mux::reloadConfig> * dynamic_reconfigure_server;
  dynamic_reconfigure::Server<stamped_cmd_vel_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(stamped_cmd_vel_mux::reloadConfig &config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to cmd_vel callback
  class CmdVelFunctor
  {
  private:
    unsigned int idx;
    StampedCmdVelMuxNodelet* node;

  public:
    CmdVelFunctor(unsigned int idx, StampedCmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::Twist::ConstPtr& msg)
    {
      node->cmdVelCallback(msg, idx);
    }

  };

  class StampedCmdVelFunctor
  {
  private:
    unsigned int idx;
    StampedCmdVelMuxNodelet* node;

  public:
    StampedCmdVelFunctor(unsigned int idx, StampedCmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
      node->stampedCmdVelCallback(msg, idx);
    }
  };

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor
  {
  private:
    unsigned int idx;
    StampedCmdVelMuxNodelet* node;

  public:
    TimerFunctor(unsigned int idx, StampedCmdVelMuxNodelet* node) :
        idx(idx), node(node)
    {
    }

    void operator()(const ros::TimerEvent& event)
    {
      node->timerCallback(event, idx);
    }
  };
};

} // namespace stamped_cmd_vel_mux

#endif /* STAMPED_CMD_VEL_MUX_HPP_ */
