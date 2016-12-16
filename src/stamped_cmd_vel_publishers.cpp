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
#include "stamped_cmd_vel_mux/stamped_cmd_vel_publishers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void StampedCmdVelPublishers::StampedCmdVelPubs::operator<<(const YAML::Node &node) {
  // Fill attributes with a YAML node content

  double new_timeout;
  std::string new_topic;
  int new_msg_type;
  node["name"] >> name;
  node["topic"] >> new_topic;
  node["msg_type"] >> new_msg_type;

#ifdef HAVE_NEW_YAMLCPP
  if (node["short_desc"]) {
#else
  if (node.FindValue("short_desc") != NULL) {
#endif
    node["short_desc"] >> short_desc;
  }

  if (new_topic != topic  || new_msg_type != msg_type) {
    // Shutdown the topic if the name has changed so it gets recreated on
    // configuration reload
    // In the case of new publishers, topic is empty and shutdown has just no
    // effect
    topic = new_topic;
    msg_type = new_msg_type;
    pub.shutdown();
  }

}

void StampedCmdVelPublishers::configure(const YAML::Node &node) {
  try {
    if (node.size() == 0) {
      throw EmptyCfgException("Configuration is empty");
    }

    std::vector<std::shared_ptr<StampedCmdVelPubs>> new_list(node.size());
    for (unsigned int i = 0; i < node.size(); i++) {
      // Parse entries on YAML

      std::string new_pubs_name = node[i]["name"].Scalar();

      auto old_pubs = std::find_if(
          list.begin(), list.end(),
          [&new_pubs_name](const std::shared_ptr<StampedCmdVelPubs> &pubs) {
            return pubs->name == new_pubs_name;
          });

      if (old_pubs != list.end()) {
        // For names already in the subscribers list, retain current object so
        // we don't re-subscribe to the topic
        new_list[i] = *old_pubs;
      } else {
        new_list[i] = std::make_shared<StampedCmdVelPubs>(i);
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


void StampedCmdVelPublishers::publishTwist( const geometry_msgs::Twist::ConstPtr& msg)
{
    static geometry_msgs::TwistStamped ts;

    for (unsigned int i = 0; i < list.size(); i++) {
      if (!list[i]->pub) {

          // Choose the topic type that we are subscribing to
          if (list[i]->msg_type)
          {
               ts.header.stamp = ros::Time::now();
               ts.twist = *msg;
               list[i]->pub.publish(ts);
          }
          else
          {
              list[i]->pub.publish(msg);
          }
      }
    }
}

void StampedCmdVelPublishers::publishTwistStamped( const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    for (unsigned int i = 0; i < list.size(); i++) {
      if (!list[i]->pub) {

          // Choose the topic type that we are subscribing to
          if (list[i]->msg_type)
          {
               list[i]->pub.publish(msg);
          }
          else
          {
              list[i]->pub.publish(msg->twist);
          }
      }
    }

}

} // namespace stamped_cmd_vel_mux
