/**
 * @file /src/stamped_cmd_vel_subscribers.cpp
 *
 * @brief  Subscriber handlers for the stamped_cmd_vel_mux
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>

#include "stamped_cmd_vel_mux/exceptions.hpp"
#include "stamped_cmd_vel_mux/stamped_cmd_vel_subscribers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void StampedCmdVelSubscribers::StampedCmdVelSubs::operator<<(const YAML::Node &node) {
  // Fill attributes with a YAML node content
  std::cout << "   inside << " << node <<  std::endl;
  double new_timeout;
  std::string new_topic;
  node["name"] >> name;
  node["topic"] >> new_topic;
  node["timeout"] >> new_timeout;
  node["msg_type"] >> msg_type;
  node["priority"] >> priority;
#ifdef HAVE_NEW_YAMLCPP
  if (node["short_desc"]) {
#else
  if (node.FindValue("short_desc") != NULL) {
#endif
    node["short_desc"] >> short_desc;
  }

  if (new_topic != topic) {
    // Shutdown the topic if the name has changed so it gets recreated on
    // configuration reload
    // In the case of new subscribers, topic is empty and shutdown has just no
    // effect
    topic = new_topic;
    subs.shutdown();
  }

  if (new_timeout != timeout) {
    // Change timer period if the timeout changed
    timeout = new_timeout;
    timer.setPeriod(ros::Duration(timeout));
  }
}

void StampedCmdVelSubscribers::configure(const YAML::Node &node) {
  try {
    if (node.size() == 0) {
      throw EmptyCfgException("Configuration is empty");
    }

    std::vector<std::shared_ptr<StampedCmdVelSubs>> new_list(node.size());
    for (unsigned int i = 0; i < node.size(); i++) {
      // Parse entries on YAML
      std::string new_subs_name = node[i]["name"].Scalar();
      auto old_subs = std::find_if(
          list.begin(), list.end(),
          [&new_subs_name](const std::shared_ptr<StampedCmdVelSubs> &subs) {
            return subs->name == new_subs_name;
          });
      if (old_subs != list.end()) {
        // For names already in the subscribers list, retain current object so
        // we don't re-subscribe to the topic
        new_list[i] = *old_subs;
      } else {
        new_list[i] = std::make_shared<StampedCmdVelSubs>(i);
      }
      // update existing or new object with the new configuration
      *new_list[i] << node[i];
    }

    list = new_list;
  } catch (EmptyCfgException &e) {
    throw e;
  } catch (YAML::ParserException &e) {
    throw YamlException(e.what());
  } catch (YAML::RepresentationException &e) {
    throw YamlException(e.what());
  }
}

} // namespace stamped_cmd_vel_mux
