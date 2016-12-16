/**
 * @file /include/stamped_cmd_vel_mux/stamped_cmd_vel_publishers.hpp
 *
 * @brief Structure for the stamped_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef STAMPED_CMD_VEL_PUBLISHERS_HPP_
#define STAMPED_CMD_VEL_PUBLISHERS_HPP_

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
** CmdVelPublishers
*****************************************************************************/

/**
 * Pool of cmd_vel topics subscribers
 */
class StampedCmdVelPublishers {
public:
  /**
   * Inner class describing an individual publishers to a cmd_vel topic
   */
  class StampedCmdVelPubs {
  public:
    unsigned int    idx;        /**< Index; assigned according to the order on YAML file */
    std::string     name;       /**< Descriptive name; must be unique to this subscriber */
    std::string     topic;      /**< The name of the topic */
    ros::Publisher  pub;        /**< The publisher itself */
    int             msg_type;   /**< Message type (0=Twist, 1=TwistStamped) */
    std::string     short_desc; /**< Short description (optional) */
    bool            active;     /**< Whether this source is active */

    StampedCmdVelPubs(unsigned int idx)
        : idx(idx), active(false), msg_type(0){};
    ~StampedCmdVelPubs() {}

    /** Fill attributes with a YAML node content */
    void operator<<(const YAML::Node &node);
  };

  StampedCmdVelPublishers() {}
  ~StampedCmdVelPublishers() {}

  std::vector<std::shared_ptr<StampedCmdVelPubs>>::size_type size() {
    return list.size();
  };
  std::shared_ptr<StampedCmdVelPubs> &operator[](unsigned int idx) {
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

  void publishTwist( const geometry_msgs::Twist::ConstPtr& msg);
  void publishTwistStamped( const geometry_msgs::TwistStamped::ConstPtr& msg);

  unsigned int allowed;

private:
  std::vector<std::shared_ptr<StampedCmdVelPubs>> list;
};

} // namespace stamped_cmd_vel_mux

#endif /* STAMPED_CMD_VEL_PUBLISHERS_HPP_ */
