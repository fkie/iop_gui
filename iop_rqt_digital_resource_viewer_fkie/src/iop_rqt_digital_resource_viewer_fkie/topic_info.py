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


import rospy
import rosgraph


class TopicInfo(object):
    publishers_idx, subscribers_idx, services_idx = range(3)
    '''
    Class to get topic and service information from ros master.

    The constructor will try to retrieve topic informations and will fail
    silently if the master is not running.
    '''
    def __init__(self):
        self._master = rosgraph.Master(rospy.get_name())
        try:
            self.update()
        except Exception:
            self._system_state = [[], [], []]
            self._topic_types = []

    def fill_published_topics(self, container, topic_type, first_topic=None):
        '''
        Fill published topics of type topic_type into container, preceded by
        first_topic.
        Convenience method to fill a QComboBox.
        @param container the container to fill. container must provide the
        methods "clear()" and "addItems()"
        @param topic_type the type of the topic. This can be a message type or
        a string.
        @param first_topic additional item which is placed at first position.
        Usually this is the current configured topic.
        '''
        self.fill_topics(container, topic_type, TopicInfo.publishers_idx, first_topic)

    def fill_topics(self, container, topic_type, idx, first_topic=None):
        container.clear()
        topics = self._get_topics(topic_type, idx)
        topics.sort()
        if first_topic is not None:
            topics.insert(0, first_topic)
        container.addItems(topics)

    def _get_topics(self, topic_type, idx):
        try:
            topic_str = topic_type._type  # access to "private" field, may fail in future ros releases
        except AttributeError:
            if (isinstance(topic_type, str)):
                topic_str = topic_type
            else:
                raise Exception("Invalid argument: topic_type must be a ros message type or string.")
        val = [t[0] for t in self._system_state[idx] if [t[0], topic_str] in self._topic_types]
        return val

    def update(self):
        '''
        Retrieve topic informations from master.

        If the master cannot be contacted an exception is raised.
        '''
        self._system_state = self._master.getSystemState()
        self._topic_types = self._master.getTopicTypes()
