/**
 * @file /include/stamped_cmd_vel_mux/stamped_cmd_vel_subscribers.hpp
 *
 * @brief Structure for the stamped_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef STAMPED_CMD_VEL_SUBSCRIBERS_HPP_
#define STAMPED_CMD_VEL_SUBSCRIBERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#ifdef HAVE_NEW_YAMLCPP
#ifndef HAVE_NEW_YAMLCPP_OPERATOR
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T> void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}
#define HAVE_NEW_YAMLCPP_OPERATOR
#endif
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
** CmdVelSubscribers
*****************************************************************************/

/**
 * Pool of cmd_vel topics subscribers
 */
class StampedCmdVelSubscribers {
public:
  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  class StampedCmdVelSubs {
  public:
    unsigned int        idx;        /**< Index; assigned according to the order on YAML file */
    std::string         name;       /**< Descriptive name; must be unique to this subscriber */
    std::string         topic;      /**< The name of the topic */
    ros::Subscriber     subs;       /**< The subscriber itself */
    ros::Timer          timer;      /**< No incoming messages timeout */
    int                 msg_type;   /**< Message type (0=Twist, 1=TwistStamped) */
    double              timeout;    /**< Timer's timeout, in seconds  */
    unsigned int        priority;   /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string         short_desc; /**< Short description (optional) */
    bool                active;     /**< Whether this source is active */

    StampedCmdVelSubs(unsigned int idx)
        : idx(idx), active(false), msg_type(0){};
    ~StampedCmdVelSubs() {}

    /** Fill attributes with a YAML node content */
    void operator<<(const YAML::Node &node);
  };

  StampedCmdVelSubscribers() {}
  ~StampedCmdVelSubscribers() {}

  std::vector<std::shared_ptr<StampedCmdVelSubs>>::size_type size() {
    return list.size();
  };
  std::shared_ptr<StampedCmdVelSubs> &operator[](unsigned int idx) {
    return list[idx];
  };

  /**
   * @brief Configures the subscribers from a yaml file.
   *
   * @exception FileNotFoundException : yaml file not found
   * @exception YamlException : problem parsing the yaml
   * @exception EmptyCfgException : empty configuration file
   * @param node : node holding all the subscriber configuration
   */
  void configure(const YAML::Node &node);

  unsigned int allowed;

private:
  std::vector<std::shared_ptr<StampedCmdVelSubs>> list;
};

} // namespace stamped_cmd_vel_mux

#endif /* STAMPED_CMD_VEL_SUBSCRIBERS_HPP_ */
