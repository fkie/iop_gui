# ROS/IOP Bridge
# Copyright (c) 2017 Fraunhofer
#
# This program is dual licensed; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation, or
# enter into a proprietary license agreement with the copyright
# holder.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; or you can read the full license at
# <http://www.gnu.de/documents/gpl-2.0.html>
#
# :author: Alexander Tiderko

from iop_msgs_fkie.msg import OcuControlFeedback


class Robot(object):

    def __init__(self, btn, sub_id, btn_state, btn_msg, group, select_state, connection_state, name):
        self._button = btn
        self._button_state = btn_state
        self._button_msg = btn_msg
        self._group = group
        self.select_state = select_state
        self.con_msg = OcuControlFeedback()
        self.con_msg.access_state = connection_state
        self._name = name
        self._subsystem_id = sub_id

    def get_robot(self):
        # return the robot
        return (self._button, self._status, self._btntext)

    def get_robot_subsytem_id(self):
        # return the subsystem_id of the robot
        return (self._subsystem_id)

    def get_state_button(self):
        # return the state button for the robot
        return (self._button_state)

    def get_msg_button(self):
        # return the components button for the robot
        return (self._button_msg)

    def get_button(self):
        # return the Button for the robot
        return (self._button)

    def get_group(self):
        # return the group for the robot
        return (self._group)

    def get_name(self):
        # return the name for the robot
        return (self._name)

    def set_name(self, name):
        # set name the name for the robot
        self._name = name

    def get_connection_msg(self):
        # return the state message
        return (self.con_msg)

    def set_connection_msg(self, msg):
        # set the state message
        self.con_msg = msg

    def set_status(self, selectstate):
        # set bool for the state of the robot select
        self.select_state = selectstate

    def get_state(self):
        # return the button state
        return self.select_state
