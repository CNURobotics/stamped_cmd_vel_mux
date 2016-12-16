stamped_cmd_vel_mux
=========================

This package handles takes multiple subscriptions to a cmd_vel topic and publishes
the active topic with the highest priority.  This version permits the use of
TwistStamped messages in addition to the standard Twist messages.

The code is based on the yocs_cmd_vel_mux ( 0.8.1 version 2016-12-15) from https://github.com/yujinrobot/yujin_ocs.git.

This version uses boost threads to reduce dependencies on external libraries.
