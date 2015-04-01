#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: users_table
   :platform: Unix
   :synopsis: A database of users.


This module provides a class that acts as a database (dictionary style) of
some set of users.

----

"""
##############################################################################
# Imports
##############################################################################

from . import users
from .exceptions import InvalidInteraction

##############################################################################
# Classes
##############################################################################


class UsersTable(object):
    '''
      The runtime populated users table along with methods to
      manipulate it.
    '''
    __slots__ = [
        '_users',  # rocon_interactions.interactions.User[]
    ]

    def __init__(self):
        """
        Constructs an empty users table.
        """
        self._users = []

    #TODO: maybe remove this method, for security reasons
    # We don't want to expose all users available
    def users(self):
        '''
          List stored users.

          :returns: a list of all roles
          :rtype: str[]
        '''
        # uniquify the list
        return list(set([u.name for u in self._users]))

    def roles(self, user=None):
        '''
          List roles filtered by user.

          :returns: a list of all roles
          :rtype: str[]
        '''
        if user:
            return list(
                set([u.role for u in self._users if u.name == user]))
        else:
            return list(set([u.role for u in self._users]))

    def __len__(self):
        return len(self._users)

    def __str__(self):
        """
        Convenient string representation of the table.
        """
        # TODO: add proper string representation of user_table
        return "User-roles table to string"

    def load(self, msgs):
        '''
          Load some users into the users table.

          :param msgs: a list of user specifications to populate the
                       table with.
          :type msgs: rocon_interaction_msgs.User_ []
          :returns: list of all additions and any that were flagged as invalid
          :rtype: (:class:`.User` [],
                   rocon_interaction_msgs.User_ []) : (new, invalid)
        '''
        new = []
        invalid = []
        for msg in msgs:
            try:
                user = users.User(msg)
                self._users.append(user)
                self._users = list(set(self._users))
                new.append(user)
            except InvalidInteraction:
                invalid.append(msg)
        return new, invalid

    def unload(self, msgs):
        '''
          Remove the specified users from the users table.

          :param msgs: a list of users
          :type msgs: rocon_authorization.User_ []

          :returns: a list of removed users
          :rtype: rocon_authorization.User_ []
        '''
        removed = []
        for msg in msgs:
            found = next((user for user in self._users
                          if user.name == msg.name and user.role == msg.role),
                         None)
            if found is not None:
                removed.append(msg)
                self._users.remove(found)
        return removed
