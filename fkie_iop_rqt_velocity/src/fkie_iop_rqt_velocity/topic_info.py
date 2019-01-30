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

import rospy
import rosgraph
import roslib
import roslib.message

def master_get_published_topics(topic_type):
    ti = TopicInfo()
    return ti.get_published_topics(topic_type)

def master_get_subscribed_topics(topic_type):
    ti = TopicInfo()
    return ti.get_subscribed_topics(topic_type)

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
        except:
            self._system_state = [ [], [], [] ]
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
    def fill_subscribed_topics(self, container, topic_type, first_topic=None):
        '''
        Fill subscribed topics of type topic_type into container, preceded by
        first_topic.
        Convenience method to fill a QComboBox.
        @param container the container to fill. container must provide the
        methods "clear()" and "addItems()"
        @param topic_type the type of the topic. This can be a message type or
        a string.
        @param first_topic additional item which is placed at first position.
        Usually this is the current configured topic.
        '''
        self.fill_topics(container, topic_type, TopicInfo.subscribers_idx, first_topic)
    def fill_topics(self, container, topic_type, idx, first_topic=None):
        container.clear()
        topics = self._get_topics(topic_type, idx)
        topics.sort()
        if not first_topic is None:
            topics.insert(0, first_topic)
        container.addItems(topics)

    def _get_topics(self, topic_type, idx):
        try:
            topic_str = topic_type._type # access to "private" field, may fail in future ros releases
        except AttributeError:
            if (isinstance(topic_type, str)):
                topic_str = topic_type
            else:
                raise Exception("Invalid argument: topic_type must be a ros message type or string.")
        val = [t[0] for t in self._system_state[idx] if [t[0],topic_str] in self._topic_types]
        return val

    def get_published_topics(self, topic_type):
        '''
        Returns all published topics of type topic_type. The topics information
        from the last call to update() is used.  
        @param topic_type the type of the topic. This can be a message type or
        a string.
        '''
        return self._get_topics(topic_type, TopicInfo.publishers_idx)

    def get_subscribed_topics(self, topic_type):
        '''
        Returns all subscribed topics of type topic_type. The topics information
        from the last call to update() is used.
        @param topic_type the type of the topic. This can be a message type or
        a string.
        '''
        return self._get_topics(topic_type, TopicInfo.subscribers_idx)

    def get_topic_type_str(self, topic):
        '''
        Returns the topic_type (str) of topic. The topics information
        from the last call to update() is used.
        @param topic the name of the topic.
        '''
        try:
            return [t[1] for t in self._topic_types if t[0] == topic][0]
        except IndexError:
            return None

    def get_topic_type(self, topic):
        '''
        Returns (topic_type, topic_type_str) of topic. The topics information
        from the last call to update() is used.
        @param topic the name of the topic.
        '''
        try:
            topic_type_str = self.get_topic_type_str(topic)
            return (roslib.message.get_message_class(topic_type_str), topic_type_str)
        except TypeError:
            return (None, None)

    def update(self):
        '''
        Retrieve topic informations from master.

        If the master cannot be contacted an exception is raised.
        '''
        self._system_state = self._master.getSystemState()
        self._topic_types = self._master.getTopicTypes()

    def publishers(self):
        '''
        Return all publishers known to the master at last call to update().
        '''
        return self._system_state[TopicInfo.publishers_idx]

    def subscribers(self):
        '''
        Return all subscribers known to the master at last call to update().
        '''
        return self._system_state[TopicInfo.subscribers_idx]

    def services(self):
        '''
        Return all services known to the master at last call to update().
        '''
        return self._system_state[TopicInfo.services_idx]

    def topic_types(self):
        return self._topic_types

    def system_state(self):
        return self._system_state

