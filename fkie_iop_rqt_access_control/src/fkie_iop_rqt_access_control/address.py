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

from fkie_iop_msgs.msg import JausAddress


class Address(object):

    def __init__(self, jaus_address):
        '''
        :type jaus_address: fkie_iop_msgs/JausMessage
        '''
        if not isinstance(jaus_address, JausAddress):
            raise TypeError("Address expects fkie_iop_msgs/JausMessage, got %s" % type(jaus_address))
        self._addr = jaus_address

    @property
    def subsystem_id(self):
        return self._addr.subsystem_id

    @property
    def node_id(self):
        return self._addr.node_id

    @property
    def component_id(self):
        return self._addr.component_id

    @property
    def msg(self):
        return self._addr

    def __repr__(self, *args, **kwargs):
        return "Address[%d.%d.%d]" % (self.subsystem_id, self.node_id, self.component_id)

    def __str__(self, *args, **kwargs):
        return "%d.%d.%d" % (self.subsystem_id, self.node_id, self.component_id)

    def __hash__(self):
        return hash((self.subsystem_id, self.node_id, self.component_id))

    def __eq__(self, other):
        return (self.subsystem_id == other.subsystem_id) and (self.node_id == other.node_id) and (self.component_id == other.component_id)

    def __ne__(self, other):
        # Not strictly necessary, but to avoid having both x==y and x!=y
        # True at the same time
        return not(self == other)

    def __lt__(self, other):
        if self.subsystem_id < other.subsystem_id:
            return True
        elif self.subsystem_id == other.subsystem_id:
            if self.node_id < other.node_id:
                return True
            elif self.node_id == other.node_id:
                if self.component_id == other.component_id:
                    return True
        return False
