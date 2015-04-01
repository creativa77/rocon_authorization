#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: manager
   :platform: Unix
   :synopsis: The ros level node class that manages user-roles relations.
----

"""
##############################################################################
# Imports
##############################################################################

import rospy

import rocon_authorization.srv as authorization_srvs

from .users_table import UsersTable
from . import users
from .exceptions import MalformedInteractionsYaml, \
    YamlResourceNotFoundException


##############################################################################
# Users
##############################################################################


class UsersManager(object):
    __slots__ = [
        '_users_table',
        '_parameters',
        '_services',
        '_watch_loop_period'
    ]

    ##########################################################################
    # Initialization
    ##########################################################################

    def __init__(self):
        self._watch_loop_period = 1.0
        self._parameters = self._setup_parameters()
        self._users_table = UsersTable()
        self._services = self._setup_services()

        # Load pre-configured users
        users_resource = self._parameters['users']
        if users_resource:
            try:
                msg_users = users.load_msgs_from_yaml_file(users_resource)
                (new_users, invalid_users) = self._users_table.load(msg_users)

                for u in new_users:
                    rospy.loginfo("Users : loading %s [%s]" %
                                  (u.name, u.role))
                for u in invalid_users:
                    rospy.logwarn("Users : failed to load %s [%s]" %
                                  (u.name, u.role))
            except YamlResourceNotFoundException as e:
                rospy.logerr("Users : failed to load resource %s [%s]" %
                             (users_resource, str(e)))
            except MalformedInteractionsYaml as e:
                rospy.logerr("Users : pre-configured users yaml malformed"
                             "[%s][%s]" % (users_resource, str(e)))

    def spin(self):
        '''
          Loop around.
        '''
        while not rospy.is_shutdown():
            rospy.rostime.wallsleep(self._watch_loop_period)

    def _setup_services(self):
        '''
          These are all public services. Typically that will drop them into the
          /concert namespace.
        '''
        services = {}
        services['get_users_roles'] = \
            rospy.Service('~get_users_roles',
                          authorization_srvs.GetUsersRoles,
                          self._ros_service_get_users_roles)
        services['set_users_roles'] = \
            rospy.Service('~set_users_roles',
                          authorization_srvs.SetUsersRoles,
                          self._ros_service_set_users_roles)
        return services

    def _setup_parameters(self):
        param = {}
        param['users'] = rospy.get_param('~users', [])
        return param

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_service_get_users_roles(self, request):
        '''
          Get roles assigned to a user.

          @param request user's name
          @type auhtorization_srvs.GetUsersRolesRequest
        '''
        user_name = request.name
        if user_name == '':
            rospy.logerr("Users: received request for roles with "
                         "empty user's name")
            role_list = []
        else:
            user_roles = self._users_table.roles(user_name)
            role_list = list(set(user_roles))
            role_list.sort()
        response = authorization_srvs.GetUsersRolesResponse()
        response.roles = role_list
        return response

    def _ros_service_set_users_roles(self, request):
        '''
          Add or remove users from the users table.

          @param request list of users-roles to set
          @type auhtorization_srvs.SetUsersRolesRequest
        '''
        if request.load:
            (new_users, invalid_users) = self._users_table.load(request.users)
            for u in new_users:
                rospy.loginfo("Users : loading %s [%s]" % (u.name, u.role))
            for u in invalid_users:
                rospy.logwarn("Users : failed to load %s [%s]" %
                              (u.name, u.role))
        else:
            removed_users = self._users_table.unload(request.users)
            for u in removed_users:
                rospy.loginfo("Users : unloading %s [%s]" % (u.name, u.role))
        # send response
        response = authorization_srvs.SetUsersRolesResponse()
        response.result = True
        return response
