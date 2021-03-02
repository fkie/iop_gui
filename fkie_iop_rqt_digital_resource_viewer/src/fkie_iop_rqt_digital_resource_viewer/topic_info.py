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

import rclpy


class TopicInfo(object):
    publishers_idx, subscribers_idx, services_idx = range(3)
    '''
    Class to get topic and service information from ros master.

    The constructor will try to retrieve topic informations and will fail
    silently if the master is not running.
    '''
    def __init__(self, node:rclpy.node.Node):
        self._node = node

    def fill_published_topics(self, container, topic_type:str, first_topic=None):
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

    def fill_subscribed_topics(self, container, topic_type:str, first_topic=None):
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

    def fill_services(self, container, service_type:str, first_service=None):
        '''
        Fill services of type service_type into container, preceded by
        first_topic.
        Convenience method to fill a QComboBox.
        @param container the container to fill. container must provide the
        methods "clear()" and "addItems()"
        @param service_type the type of the service. This can be a message type or
        a string.
        @param first_service additional item which is placed at first position.
        Usually this is the current configured topic.
        '''
        self.fill_topics(container, service_type, TopicInfo.services_idx, first_service)

    def fill_topics(self, container, topic_type:str, idx:int, first_topic=None):
        container.clear()
        topics = self._get_topics(topic_type, idx)
        topics.sort()
        if first_topic is not None:
            if not topics or topics[0] != first_topic:
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
        result = []
        nnl = self._node.get_node_names_and_namespaces()
        for nn in nnl:
            ttl = []
            if idx == TopicInfo.subscribers_idx:
                ttl = self._node.get_subscriber_names_and_types_by_node(nn[0], nn[1])
            elif idx == TopicInfo.publishers_idx:
                ttl = self._node.get_publisher_names_and_types_by_node(nn[0], nn[1])
            elif idx == TopicInfo.services_idx:
                ttl = self._node.get_service_names_and_types_by_node(nn[0], nn[1])
            for tt in ttl:
                if topic_str in tt[1]:
                    result.append(tt[0])
        return result

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

    # def get_topic_type_str(self, topic):
    #     '''
    #     Returns the topic_type (str) of topic. The topics information
    #     from the last call to update() is used.
    #     @param topic the name of the topic.
    #     '''
    #     try:
    #         return [t[1] for t in self._topic_types if t[0] == topic][0]
    #     except IndexError:
    #         return None

    # def get_topic_type(self, topic):
    #     '''
    #     Returns (topic_type, topic_type_str) of topic. The topics information
    #     from the last call to update() is used.
    #     @param topic the name of the topic.
    #     '''
    #     try:
    #         topic_type_str = self.get_topic_type_str(topic)
    #         return (roslib.message.get_message_class(topic_type_str), topic_type_str)
    #     except TypeError:
    #         return (None, None)

    # def publishers(self):
    #     '''
    #     Return all publishers known to the master at last call to update().
    #     '''
    #     return self._system_state[TopicInfo.publishers_idx]

    # def subscribers(self):
    #     '''
    #     Return all subscribers known to the master at last call to update().
    #     '''
    #     return self._system_state[TopicInfo.subscribers_idx]

    # def services(self):
    #     '''
    #     Return all services known to the master at last call to update().
    #     '''
    #     return self._system_state[TopicInfo.services_idx]

    # def topic_types(self):
    #     return self._topic_types

    # def system_state(self):
    #     return self._system_state
