#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: users
   :platform: Unix
   :synopsis: Representative class and methods for a *user*.


This module defines a class and methods that represent the core of what
a user is.

----

"""
##############################################################################
# Imports
##############################################################################

import yaml
import zlib  # crc32
import os

import genpy
import rospkg
import rocon_console.console as console
import rocon_python_utils
import rocon_authorization.msg as user_msgs

from .exceptions import InvalidInteraction, MalformedInteractionsYaml, YamlResourceNotFoundException

##############################################################################
# Utility Methods
##############################################################################

def load_msgs_from_yaml_file(file_path):
    """
      Load users from a yaml file.

      :param str file_path: file path of a yaml formatted interactions file (ext=.interactions).

      :returns: a list of ros msg user specifications
      :rtype: rocon_authorization_msgs.User_ []

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.
    """
    users = []
    try:
        yaml_filename = file_path
        if not os.path.isfile(yaml_filename):
            raise YamlResourceNotFoundException(str(e))
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))
    with open(yaml_filename) as f:
        # load the users from yaml into a python object
        user_yaml_objects = yaml.load(f)
        # now drop it into message format
        for user_yaml_object in user_yaml_objects:
            # convert the parameters from a freeform yaml variable to a yaml string suitable for
            # shipping off in ros msgs (where parameters is a string variable)
            if 'parameters' in user_yaml_object:  # it's an optional key
                # chomp trailing newlines
                user_yaml_object['parameters'] = yaml.dump(user_yaml_object['parameters']).rstrip()
            user = user_msgs.User()
            try:
                genpy.message.fill_message_args(user, user_yaml_object)
            except genpy.MessageException as e:
                raise MalformedInteractionsYaml(
                    "malformed yaml preventing converting of yaml to user msg type [%s]" % str(e))
            users.append(user)
    return users

def load_msgs_from_yaml_resource(resource_name):
    """
      Load users from a yaml resource.

      :param str resource_name: pkg/filename of a yaml formatted users file (ext=.users).

      :returns: a list of ros msg users specifications
      :rtype: rocon_authorization_msgs.User_ []

      :raises: :exc:`.YamlResourceNotFoundException` if yaml is not found.
      :raises: :exc:`.MalformedInteractionsYaml` if yaml is malformed.
    """
    users = []
    try:
        yaml_filename = rocon_python_utils.ros.find_resource_from_string(resource_name, extension='users')
        users = load_msgs_from_yaml_file(yaml_filename)
        return users
    except rospkg.ResourceNotFound as e:  # resource not found.
        raise YamlResourceNotFoundException(str(e))


##############################################################################
# Classes
##############################################################################

class User(object):

    '''
      This class defines an user. It does so by wrapping the base
      rocon_authorization_msgs.User_ msg structure with
      a few convenient variables and methods.

      .. include:: weblinks.rst
    '''
    __slots__ = [
        'msg',           # rocon_authorization_msgs.User
    ]

    def __init__(self, msg):
        """
          :param msg: underlying data structure with fields minimally filled via :func:`.load_msgs_from_yaml_resource`.
          :type msg: rocon_authorization_msgs.User_
        """
        self.msg = msg

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def name(self):
        """Name of the user."""
        return self.msg.user

    @property
    def role(self):
        """User's role."""
        return self.msg.role

    def _eq__(self, other):
        if type(other) is type(self):
            return self.msg.user == other.msg.user and self.msg.role == other.msg.role
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        '''
          Format the user-role relation into a human-readable string.
        '''
        s += console.cyan + "  Name" + console.reset + "         : " + console.yellow + "%s" % self.msg.user + console.reset + '\n'  # noqa
        s += console.cyan + "  Role" + console.reset + "  : " + console.yellow + "%s" % self.msg.role + console.reset + '\n'  # noqa

        return s
