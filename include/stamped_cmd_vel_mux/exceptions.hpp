/**
 * @file /cmd_vel_mux/include/stamped_cmd_vel_mux/exceptions.hpp
 *
 * @brief Exception classes for cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/CNURobotics/stamped_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef STAMPED_CMD_VEL_EXCEPTIONS_HPP_
#define STAMPED_CMD_VEL_EXCEPTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <exception>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace stamped_cmd_vel_mux {

/*****************************************************************************
** Exceptions
*****************************************************************************/

class FileNotFoundException : public std::runtime_error {
public:
  FileNotFoundException(const std::string &msg) : std::runtime_error(msg) {}
  virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException : public std::runtime_error {
public:
  EmptyCfgException(const std::string &msg) : std::runtime_error(msg) {}
  virtual ~EmptyCfgException() throw() {}
};

class YamlException : public std::runtime_error {
public:
  YamlException(const std::string &msg) : std::runtime_error(msg) {}
  virtual ~YamlException() throw() {}
};

} // namespace stamped_cmd_vel_mux

#endif /* STAMPED_CMD_VEL_EXCEPTIONS_HPP_ */
