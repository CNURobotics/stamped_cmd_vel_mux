/**
 * @file /src/stamped_cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include "stamped_cmd_vel_mux/stamped_cmd_vel_mux_nodelet.hpp"
#include "stamped_cmd_vel_mux/exceptions.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void StampedCmdVelMuxNodelet::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr &msg, unsigned int idx) {
  // Reset general timer
  common_timer.stop();
  common_timer.start();

  // Reset timer for this source
  cmd_vel_subs[idx]->timer.stop();
  cmd_vel_subs[idx]->timer.start();

  cmd_vel_subs[idx]->active =
      true; // obviously his source is sending commands, so active

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((cmd_vel_subs.allowed == VACANT) || (cmd_vel_subs.allowed == idx) ||
      (cmd_vel_subs[idx]->priority >
       cmd_vel_subs[cmd_vel_subs.allowed]->priority)) {
    if (cmd_vel_subs.allowed != idx) {
      cmd_vel_subs.allowed = idx;

      // Notify the world that a new cmd_vel source took the control
      std_msgs::StringPtr acv_msg(new std_msgs::String);
      acv_msg->data = cmd_vel_subs[idx]->name;
      active_subscriber.publish(acv_msg);
    }

    cmd_vel_pubs.publishTwist(msg);
  }
}
void StampedCmdVelMuxNodelet::stampedCmdVelCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg, unsigned int idx) {
  // Reset general timer
  common_timer.stop();
  common_timer.start();

  // Reset timer for this source
  cmd_vel_subs[idx]->timer.stop();
  cmd_vel_subs[idx]->timer.start();

  cmd_vel_subs[idx]->active =
      true; // obviously his source is sending commands, so active

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((cmd_vel_subs.allowed == VACANT) || (cmd_vel_subs.allowed == idx) ||
      (cmd_vel_subs[idx]->priority >
       cmd_vel_subs[cmd_vel_subs.allowed]->priority)) {
    if (cmd_vel_subs.allowed != idx) {
      cmd_vel_subs.allowed = idx;

      // Notify the world that a new cmd_vel source took the control
      std_msgs::StringPtr acv_msg(new std_msgs::String);
      acv_msg->data = cmd_vel_subs[idx]->name;
      active_subscriber.publish(acv_msg);
    }

    cmd_vel_pubs.publishTwistStamped(msg);
  }
}

void StampedCmdVelMuxNodelet::timerCallback(const ros::TimerEvent &event,
                                            unsigned int idx) {
  if (cmd_vel_subs.allowed == idx ||
      (idx == GLOBAL_TIMER && cmd_vel_subs.allowed != VACANT)) {
    if (idx == GLOBAL_TIMER) {
      // No cmd_vel messages timeout happened for ANYONE, so last active source
      // got stuck without further
      // messages; not a big problem, just dislodge it; but possibly reflect a
      // problem in the controller
      NODELET_WARN(
          "StampedCmdVelMux : No cmd_vel messages from ANY input received in "
          "the last %fs",
          common_timer_period);
      NODELET_WARN("StampedCmdVelMux : %s dislodged due to general timeout",
                   cmd_vel_subs[cmd_vel_subs.allowed]->name.c_str());
    }

    // No cmd_vel messages timeout happened to currently active source, so...
    cmd_vel_subs.allowed = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "idle";
    active_subscriber.publish(acv_msg);
  }

  if (idx != GLOBAL_TIMER)
    cmd_vel_subs[idx]->active = false;
}

void StampedCmdVelMuxNodelet::onInit() {
  ros::NodeHandle &nh = this->getPrivateNodeHandle();

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure_cb =
      boost::bind(&StampedCmdVelMuxNodelet::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server =
      new dynamic_reconfigure::Server<stamped_cmd_vel_mux::reloadConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

  active_subscriber =
      nh.advertise<std_msgs::String>("active", 1, true); // latched topic

  // Notify the world that by now nobody is publishing on cmd_vel yet
  std_msgs::StringPtr active_msg(new std_msgs::String);
  active_msg->data = "idle";
  active_subscriber.publish(active_msg);

  // could use a call to reloadConfiguration here, but it seems to automatically
  // call it once with defaults anyway.
  NODELET_DEBUG("StampedCmdVelMux : successfully initialized");
}

void StampedCmdVelMuxNodelet::reloadConfiguration(
    stamped_cmd_vel_mux::reloadConfig &config, uint32_t unused_level) {
  ros::NodeHandle &pnh = this->getPrivateNodeHandle();

  std::unique_ptr<std::istream> is;

  // Configuration can come directly as a yaml-formatted string or as a file
  // path,
  // but not both, so we give priority to the first option
  if (config.yaml_cfg_data.size() > 0) {
    is.reset(new std::istringstream(config.yaml_cfg_data));
  } else {
    std::string yaml_cfg_file;
    if (config.yaml_cfg_file == "") {
      // typically fired on startup, so look for a parameter to set a default
      pnh.getParam("yaml_cfg_file", yaml_cfg_file);
    } else {
      yaml_cfg_file = config.yaml_cfg_file;
    }

    is.reset(new std::ifstream(yaml_cfg_file.c_str(), std::ifstream::in));
    if (is->good() == false) {
      NODELET_ERROR_STREAM("StampedCmdVelMux : configuration file not found ["
                           << yaml_cfg_file << "]");
      return;
    }
  }

  /*********************
  ** Yaml File Parsing
  **********************/

  // probably need to bring the try catches back here
  YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(*is);
#else
  YAML::Parser parser(*is);
  parser.GetNextDocument(doc);
#endif

  /*********************
  ** Output Publishers
  **********************/
  try {
    cmd_vel_pubs.configure(doc["publishers"]);
  } catch (EmptyCfgException &e) {
    NODELET_WARN_STREAM("StampedCmdVelMux : yaml configured zero publishers, "
                        "check yaml content");
  } catch (YamlException &e) {
    NODELET_ERROR_STREAM("StampedCmdVelMux : yaml parsing problem ["
                         << std::string(e.what()) << "]");
  }

  for (unsigned int i = 0; i < cmd_vel_pubs.size(); i++) {
    if (!cmd_vel_pubs[i]->pub) {

        // Choose the topic type that we are subscribing to
        if (cmd_vel_pubs[i]->msg_type)
        {
             cmd_vel_pubs[i]->pub = pnh.advertise<geometry_msgs::TwistStamped>(cmd_vel_pubs[i]->topic, 10);
        }
        else
        {
            cmd_vel_pubs[i]->pub = pnh.advertise<geometry_msgs::Twist>(cmd_vel_pubs[i]->topic, 10);
        }
        NODELET_DEBUG("StampedCmdVelMux : publishing to '%s' on topic '%s' msg_type: %d ",
                    cmd_vel_subs[i]->name.c_str(),
                    cmd_vel_subs[i]->topic.c_str(),
                    cmd_vel_subs[i]->msg_type);
    } else {
      NODELET_DEBUG_STREAM(
          "StampedCmdVelMux : no need to re-advertise topic '"
          << cmd_vel_pubs[i]->topic << "'");
    }
  }


  /*********************
  ** Input Subscribers
  **********************/
  try {
    cmd_vel_subs.configure(doc["subscribers"]);
  } catch (EmptyCfgException &e) {
    NODELET_WARN_STREAM("StampedCmdVelMux : yaml configured zero subscribers, "
                        "check yaml content");
  } catch (YamlException &e) {
    NODELET_ERROR_STREAM("StampedCmdVelMux : yaml parsing problem ["
                         << std::string(e.what()) << "]");
  }

  // (Re)create subscribers whose topic is invalid: new ones and those with
  // changed names
  double longest_timeout = 0.0;
  for (unsigned int i = 0; i < cmd_vel_subs.size(); i++) {
    if (!cmd_vel_subs[i]->subs) {

        // Choose the topic type that we are subscribing to
        if (cmd_vel_subs[i]->msg_type)
        {
             cmd_vel_subs[i]->subs = pnh.subscribe<geometry_msgs::TwistStamped>(
                cmd_vel_subs[i]->topic, 10, StampedCmdVelFunctor(i, this));
        }
        else
        {
            cmd_vel_subs[i]->subs = pnh.subscribe<geometry_msgs::Twist>(
               cmd_vel_subs[i]->topic, 10, CmdVelFunctor(i, this));
        }
        NODELET_DEBUG("StampedCmdVelMux : subscribed to '%s' on topic '%s'. pr: "
                    "%d, to: %.2f msg_typ: %d ",
                    cmd_vel_subs[i]->name.c_str(),
                    cmd_vel_subs[i]->topic.c_str(), cmd_vel_subs[i]->priority,
                    cmd_vel_subs[i]->timeout,
                    cmd_vel_subs[i]->msg_type);
    } else {
      NODELET_DEBUG_STREAM(
          "StampedCmdVelMux : no need to re-subscribe to input topic '"
          << cmd_vel_subs[i]->topic << "'");
    }

    if (!cmd_vel_subs[i]->timer) {
      // Create (stopped by now) a one-shot timer for every subscriber, if it
      // doesn't exist yet
      cmd_vel_subs[i]->timer =
          pnh.createTimer(ros::Duration(cmd_vel_subs[i]->timeout),
                          TimerFunctor(i, this), true, false);
    }

    if (cmd_vel_subs[i]->timeout > longest_timeout)
      longest_timeout = cmd_vel_subs[i]->timeout;
  }

  if (!common_timer) {
    // Create another timer for cmd_vel messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    common_timer_period = longest_timeout * 2.0;
    common_timer =
        pnh.createTimer(ros::Duration(common_timer_period),
                        TimerFunctor(GLOBAL_TIMER, this), true, false);
  } else if (longest_timeout != (common_timer_period / 2.0)) {
    // Longest timeout changed; just update existing timer period
    common_timer_period = longest_timeout * 2.0;
    common_timer.setPeriod(ros::Duration(common_timer_period));
  }

  NODELET_INFO_STREAM("StampedCmdVelMux : (re)configured");
}

} // namespace stamped_cmd_vel_mux

PLUGINLIB_EXPORT_CLASS(stamped_cmd_vel_mux::StampedCmdVelMuxNodelet,
                       nodelet::Nodelet);
