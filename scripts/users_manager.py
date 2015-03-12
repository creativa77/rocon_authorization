#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_authorization

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':

    rospy.init_node('rocon_authorization')
    users_manager = rocon_authorization.UsersManager()
    users_manager.spin()
