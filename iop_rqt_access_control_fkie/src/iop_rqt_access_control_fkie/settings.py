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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, Signal
try:
    from python_qt_binding.QtGui import QComboBox, QDialog
except:
    from python_qt_binding.QtWidgets import QComboBox, QDialog

import os
import rospy
import time
from std_srvs.srv import Empty
from iop_msgs_fkie.msg import Identification, OcuFeedback, System, OcuCmd, HandoffRequest, HandoffResponse
from .topic_info import TopicInfo


class Settings(QObject):
    '''
    Provides a configuration dialog and topic management.
    '''
    signal_system = Signal(System)
    signal_feedback = Signal(OcuFeedback)
    signal_ident = Signal(Identification)
    signal_handoff_request = Signal(HandoffRequest)
    signal_handoff_response = Signal(HandoffResponse)

    def __init__(self):
        QObject.__init__(self)

        self.dialog_config = QDialog()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_access_control_config.ui')
        loadUi(ui_file, self.dialog_config)
        self.dialog_config.accepted.connect(self.on_dialog_config_accepted)
        self._ti = TopicInfo()

        self._topic_system = '/iop_system'
        self._service_update_discovery = '/iop_update_discovery'
        self._topic_identification = '/iop_identification'
        self._topic_cmd = '/ocu_cmd'
        self._topic_feedback = '/ocu_feedback'
        self._topic_handoff_own_request = '/handoff_own_request'
        self._topic_handoff_own_response = '/handoff_own_response'
        self._topic_handoff_remote_request = '/handoff_remote_request'
        self._topic_handoff_remote_response = '/handoff_remote_response'
        self._authority = 205
        self._namespace = '/'
        self._handoff_autorequest = False
        self._handoff_explanation = 'autorequest'

        self._pub_cmd = None
        self._sub_feedback = None
        self._sub_ident = None
        self._sub_system = None
        self._pub_handoff_own_request = None
        self._pub_handoff_own_response = None
        self._sub_handoff_remote_request = None
        self._sub_handoff_remote_response = None

    @property
    def authority(self):
        return self._authority

    @property
    def handoff_autorequest(self):
        return self._handoff_autorequest

    @property
    def handoff_explanation(self):
        return self._handoff_explanation

    def publish_cmd(self, cmd):
        time.sleep(0.1)
        if self._pub_cmd is not None and not rospy.is_shutdown():
            self._pub_cmd.publish(cmd)

    def publish_handoff_request(self, msg):
        time.sleep(0.1)
        if self._pub_handoff_own_request is not None and not rospy.is_shutdown():
            self._pub_handoff_own_request.publish(msg)

    def publish_handoff_response(self, msg):
        time.sleep(0.1)
        if self._pub_handoff_own_response is not None and not rospy.is_shutdown():
            self._pub_handoff_own_response.publish(msg)

    def _callback_ocu_feedback(self, control_feedback):
        self.signal_feedback.emit(control_feedback)

    def _callback_ocu_ident(self, ident):
        self.signal_ident.emit(ident)

    def _callback_system(self, msg):
        self.signal_system.emit(msg)

    def _callback_handoff_remote_request(self, msg):
        self.signal_handoff_request.emit(msg)

    def _callback_handoff_remote_response(self, msg):
        self.signal_handoff_response.emit(msg)

    def update_discovery(self):
        '''
        Currently there are no interface to update the subsystem
        '''
        # send query identification to update the system
        srvs_name = self._service_update_discovery
        # rospy.wait_for_service(srvs_name)
        try:
            update_srvs = rospy.ServiceProxy(srvs_name, Empty)
            update_srvs()
        except rospy.ServiceException, e:
            print "Service call for update JAUS network failed: %s" % e

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('authority', self._authority)
        instance_settings.set_value('namespace', self._namespace)
        instance_settings.set_value('handoff_autorequest', self._handoff_autorequest)
        instance_settings.set_value('handoff_explanation', self._handoff_explanation)
        instance_settings.set_value('iop_system', self._topic_system)
        instance_settings.set_value('iop_update_discovery', self._service_update_discovery)
        instance_settings.set_value('iop_identification', self._topic_identification)
        instance_settings.set_value('ocu_cmd', self._topic_cmd)
        instance_settings.set_value('ocu_feedback', self._topic_feedback)

        instance_settings.set_value('topic_handoff_own_request', self._topic_handoff_own_request)
        instance_settings.set_value('topic_handoff_own_response', self._topic_handoff_own_response)
        instance_settings.set_value('topic_handoff_remote_request', self._topic_handoff_remote_request)
        instance_settings.set_value('topic_handoff_remote_response', self._topic_handoff_remote_response)

    def restore_settings(self, plugin_settings, instance_settings):

        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        # TODO RELEASE ALL CONNROL?
        self.shutdownRosComm()
        self._authority = (int)(instance_settings.value('authority', 205))
        self._namespace = instance_settings.value('namespace', '/')
        self._handoff_autorequest = instance_settings.value('handoff_autorequest', False)
        if isinstance(self._handoff_autorequest, (str, unicode)):
            self._handoff_autorequest = self._handoff_autorequest.lower() in ("yes", "true", "t", "1")
        self._handoff_explanation = instance_settings.value('handoff_explanation', 'autorequest')
        self._topic_system = instance_settings.value('iop_system', '/iop_system')
        self._service_update_discovery = instance_settings.value('iop_update_discovery', '/iop_update_discovery')
        self._topic_identification = instance_settings.value('iop_identification', '/iop_identification')
        self._topic_cmd = instance_settings.value('ocu_cmd', '/ocu_cmd')
        self._topic_feedback = instance_settings.value('ocu_feedback', '/ocu_feedback')
        self._topic_handoff_own_request = instance_settings.value('topic_handoff_own_request', '/handoff_own_request')
        self._topic_handoff_own_response = instance_settings.value('topic_handoff_own_response', '/handoff_own_response')
        self._topic_handoff_remote_request = instance_settings.value('topic_handoff_remote_request', '/handoff_remote_request')
        self._topic_handoff_remote_response = instance_settings.value('topic_handoff_remote_response', '/handoff_remote_response')

        self.reinitRosComm()

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
        self.dialog_config = QDialog()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_access_control_config.ui')
        loadUi(ui_file, self.dialog_config)
        self.dialog_config.accepted.connect(self.on_dialog_config_accepted)
        self._current_namespace = self.dialog_config.comboBox_namespace.currentText()
        if (rospy.names.get_namespace() != self._current_namespace):
            self.dialog_config.comboBox_namespace.addItem(rospy.names.get_namespace())
        self.dialog_config.comboBox_namespace.focusOutEvent = self._namespace_focusOutEvent

        self.dialog_config.comboBox_authority.setCurrentText("%d" % self._authority)
        self.dialog_config.comboBox_namespace.setCurrentText(self._namespace)
        self.dialog_config.checkBox_autorequest.setChecked(self._handoff_autorequest)
        self.dialog_config.lineEdit_explanation.setText(self._handoff_explanation)
        # fill configuration dialog
        ti = TopicInfo()
        ti.fill_published_topics(self.dialog_config.comboBox_discovery_topic, "iop_msgs_fkie/System", self._topic_system)
        default_name = rospy.names.ns_join(rospy.get_namespace(), 'iop_update_discovery')
        if default_name != self._service_update_discovery:
            self.dialog_config.comboBox_update_service.addItem(default_name)
        ti.fill_published_topics(self.dialog_config.comboBox_identification_topic, "iop_msgs_fkie/Identification", self._topic_identification)
        ti.fill_published_topics(self.dialog_config.comboBox_cmd_topic, "iop_msgs_fkie/OcuCmd", self._topic_cmd)
        ti.fill_published_topics(self.dialog_config.comboBox_feedback_topic, "iop_msgs_fkie/OcuFeedback", self._topic_feedback)
        ti.fill_subscribed_topics(self.dialog_config.comboBox_handoff_own_request_topic, "iop_msgs_fkie/HandoffRequest", self._topic_handoff_own_request)
        ti.fill_subscribed_topics(self.dialog_config.comboBox_handoff_own_response_topic, "iop_msgs_fkie/HandoffResponse", self._topic_handoff_own_response)
        ti.fill_published_topics(self.dialog_config.comboBox_handoff_remote_request_topic, "iop_msgs_fkie/HandoffRequest", self._topic_handoff_remote_request)
        ti.fill_published_topics(self.dialog_config.comboBox_handoff_remote_response_topic, "iop_msgs_fkie/HandoffResponse", self._topic_handoff_remote_response)
        # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def _namespace_focusOutEvent(self, event):
        QComboBox.focusOutEvent(self.dialog_config.comboBox_namespace, event)
        self.test_namespace()

    def test_namespace(self):
        if self._current_namespace != self.dialog_config.comboBox_namespace.currentText():
            self._current_namespace = self.dialog_config.comboBox_namespace.currentText()
            newname = self._replace_namespace(self._service_update_discovery, self._current_namespace)
            self.dialog_config.comboBox_update_service.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_system, self._current_namespace)
            self.dialog_config.comboBox_discovery_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_identification, self._current_namespace)
            self.dialog_config.comboBox_identification_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_cmd, self._current_namespace)
            self.dialog_config.comboBox_cmd_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_feedback, self._current_namespace)
            self.dialog_config.comboBox_feedback_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_handoff_own_request, self._current_namespace)
            self.dialog_config.comboBox_handoff_own_request_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_handoff_own_response, self._current_namespace)
            self.dialog_config.comboBox_handoff_own_response_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_handoff_remote_request, self._current_namespace)
            self.dialog_config.comboBox_handoff_remote_request_topic.setCurrentText(newname)
            newname = self._replace_namespace(self._topic_handoff_remote_response, self._current_namespace)
            self.dialog_config.comboBox_handoff_remote_response_topic.setCurrentText(newname)

    def _replace_namespace(self, topic, newns):
        return rospy.names.ns_join(newns, topic.split(rospy.names.SEP)[-1])

    def on_dialog_config_accepted(self):
        self.test_namespace()
        self.shutdownRosComm()
        self._authority = (int)(self.dialog_config.comboBox_authority.currentText())
        self._namespace = self.dialog_config.comboBox_namespace.currentText()
        self._handoff_autorequest = self.dialog_config.checkBox_autorequest.isChecked()
        self._handoff_explanation = self.dialog_config.lineEdit_explanation.text()
        self._topic_system = self.dialog_config.comboBox_discovery_topic.currentText()
        self._service_update_discovery = self.dialog_config.comboBox_update_service.currentText()
        self._topic_identification = self.dialog_config.comboBox_identification_topic.currentText()
        self._topic_cmd = self.dialog_config.comboBox_cmd_topic.currentText()
        self._topic_feedback = self.dialog_config.comboBox_feedback_topic.currentText()
        self._topic_handoff_own_request = self.dialog_config.comboBox_handoff_own_request_topic.currentText()
        self._topic_handoff_own_response = self.dialog_config.comboBox_handoff_own_response_topic.currentText()
        self._topic_handoff_remote_request = self.dialog_config.comboBox_handoff_remote_request_topic.currentText()
        self._topic_handoff_remote_response = self.dialog_config.comboBox_handoff_remote_response_topic.currentText()
        self.reinitRosComm()

    def reinitRosComm(self):
        # if self._topic_system:
        #     self._topic_system = rosgraph.names.script_resolve_name('rostopic', self._topic_system)
        if self._topic_system:
            self._sub_system = rospy.Subscriber(self._topic_system, System, self._callback_system, queue_size=10)
        if self._pub_cmd is None:
            self._pub_cmd = rospy.Publisher(self._topic_cmd, OcuCmd, latch=True, queue_size=10)
        if self._sub_feedback is None:
            self._sub_feedback = rospy.Subscriber(self._topic_feedback, OcuFeedback, self._callback_ocu_feedback, queue_size=10)
        if self._sub_ident is None:
            self._sub_ident = rospy.Subscriber(self._topic_identification, Identification, self._callback_ocu_ident, queue_size=10)
        if self._pub_handoff_own_request is None:
            self._pub_handoff_own_request = rospy.Publisher(self._topic_handoff_own_request, HandoffRequest, queue_size=10)
        if self._pub_handoff_own_response is None:
            self._pub_handoff_own_response = rospy.Publisher(self._topic_handoff_own_response, HandoffResponse, queue_size=10)
        if self._sub_handoff_remote_request is None:
            self._sub_handoff_remote_request = rospy.Subscriber(self._topic_handoff_remote_request, HandoffRequest, self._callback_handoff_remote_request, queue_size=10)
        if self._sub_handoff_remote_response is None:
            self._sub_handoff_remote_response = rospy.Subscriber(self._topic_handoff_remote_response, HandoffResponse, self._callback_handoff_remote_response, queue_size=10)

    def shutdownRosComm(self):
        if self._sub_system is not None:
            self._sub_system.unregister()
            self._sub_system = None
        if self._sub_feedback is not None:
            self._sub_feedback.unregister()
            self._sub_feedback = None
        if self._sub_ident is not None:
            self._sub_ident.unregister()
            self._sub_ident = None
        if self._pub_cmd is not None:
            self._pub_cmd.unregister()
            self._pub_cmd = None
        if self._pub_handoff_own_request is not None:
            self._pub_handoff_own_request.unregister()
            self._pub_handoff_own_request = None
        if self._pub_handoff_own_response is not None:
            self._pub_handoff_own_response.unregister()
            self._pub_handoff_own_response = None
        if self._sub_handoff_remote_request is not None:
            self._sub_handoff_remote_request.unregister()
            self._sub_handoff_remote_request = None
        if self._sub_handoff_remote_response is not None:
            self._sub_handoff_remote_response.unregister()
            self._sub_handoff_remote_response = None
