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

from iop_msgs_fkie.msg import DigitalResourceEndpoints
from iop_msgs_fkie.srv import QueryByAddr
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, QUrl, Qt
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QPushButton, QMessageBox
except:
    from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QPushButton, QMessageBox

from qt_gui.plugin import Plugin
import os
import rosgraph
import roslib.message
import rospy

from iop_ocu_controllib_fkie.ocu_control_slave import OcuControlSlave

from .cams import Cam
from .topic_info import TopicInfo


# try to import phonon
try:
    from python_qt_binding.phonon import Phonon
except:
    try:
        from python_qt_binding.QtCore import QT_VERSION_STR
        from PyQt4.phonon import Phonon
    except:
        from PySide.phonon import Phonon


class DigitalResourceViewer(Plugin):

    signal_topic_endpoints = Signal(DigitalResourceEndpoints)
    signal_play = Signal()
    signal_stop = Signal()
    signal_update_entpoints = Signal()

    def __init__(self, context):
        super(DigitalResourceViewer, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DigitalResourceViewer')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        _args, _unknowns = parser.parse_known_args(context.argv())
        # Create QWidget
        self._widget = QWidget()
        self._cam_list = []
        self._current_cam = None
        self._start = False
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        # ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_digital_resource_viewer.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('DigitalResourceViewerUi')
        self._widget.setWindowTitle("DigitalResourceViewer")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.context = context
        self._topic_endpoints = rospy.names.ns_join(rospy.get_namespace(), 'endpoints')
        self._service_update_endpoints = rospy.names.ns_join(rospy.get_namespace(), 'update_endpoints')
        self._subscriber_control_platform = None
        self._subscriber_endpoints = None
        self._endpoints = None
        self.signal_topic_endpoints.connect(self.signal_callback_endpoints)
        self.signal_play.connect(self.clicked_play)
        self.signal_stop.connect(self.clicked_stop)
        self.signal_update_entpoints.connect(self.clicked_update)
        self._widget.pushButton_info.clicked.connect(self.show_info)
        self._widget.pushButton_info.setIcon(QIcon.fromTheme("help-about"))
        self._widget.pushButton_info.setStyleSheet("QPushButton{border: None;background-repeat: no-repeat;}")
        self._hbox_cam_control = QHBoxLayout()
        self._widget.frame.setLayout(self._hbox_cam_control)

        self._cam_id = 1
        # rtsp player
        self.player = Phonon.VideoPlayer(Phonon.VideoCategory, self._widget)
        self.player.finished.connect(self.player.deleteLater)
        self._widget.layout().addWidget(self.player)
        self._ocu_control_slave = OcuControlSlave()
        self._ocu_control_slave.set_control_platform_handler(self.control_platform_callback)
        self._ocu_control_slave.set_access_control_handler(self.access_control_handler)
        self._media = Phonon.MediaObject()

    def show_info(self):
        # self.clicked_update()
        if len(self._cam_list):
            info_msg = ""
            i = 1
            for cam in self._cam_list:
                info_msg = "%scam %d:  %s\n" % (info_msg, i, cam)
                i = i + 1
            self._info_window = QDialog(self._widget)
            self._info_window.setMinimumWidth(150)
            self.vlayout = QVBoxLayout(self._info_window)
            self.info_label = QLabel(info_msg)
            self.info_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.vlayout.addWidget(self.info_label)
            self._info_widget = QWidget(self._info_window)
            self._info_window.setWindowTitle("cam info")
            self._info_window.setWindowIcon(QIcon.fromTheme("help-about"))
            self._info_window.show()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.shutdownRosComm()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('topic_endpoints', self._topic_endpoints)
        instance_settings.set_value('service_update_endpoints', self._service_update_endpoints)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self.shutdownRosComm()
        self._topic_endpoints = instance_settings.value('topic_endpoints', rospy.names.ns_join(rospy.get_namespace(), 'endpoints'))
        self._service_update_endpoints = instance_settings.value('service_update_endpoints', rospy.names.ns_join(rospy.get_namespace(), 'update_endpoints'))
        self.reinitRosComm()

    def reinitRosComm(self):
        if self._topic_endpoints:
            self._topic_endpoints = rosgraph.names.script_resolve_name('rostopic', self._topic_endpoints)
            if self._topic_endpoints:
                self._subscriber_control_platform = rospy.Subscriber(self._topic_endpoints, DigitalResourceEndpoints, self.callback_endpoints)

    def shutdownRosComm(self):
        if self._subscriber_endpoints:
            self._subscriber_endpoints.unregister()
            self._subscriber_endpoints = None

    def callback_endpoints(self, msg):
        self.signal_topic_endpoints.emit(msg)

    def signal_callback_endpoints(self, msg):
        # update
        self._endpoints = msg
        self.create_cam_buttons(msg.endpoints)

    def create_cam_buttons(self, endpoints):
        if self._cam_list:
            new_cam_list = []
            for cam in self._cam_list:
                if not cam.is_in(endpoints):
                    widgetToRemove = cam.get_cam_button()
                    self._hbox_cam_control.removeWidget(widgetToRemove)
                    widgetToRemove.setParent(None)
                else:
                    new_cam_list.append(cam)
            self._cam_list = new_cam_list

        for endpoint in endpoints:
            button = QPushButton("cam %d" % self._cam_id)
            button.setFixedSize(57, 24)
            button.clicked.connect(self.activate_cam)
            new_cam = Cam(endpoint, button, False)
            if not new_cam.is_in(self._cam_list):
                self._cam_id += 1
                self._hbox_cam_control.addWidget(button)
                self._cam_list.append(new_cam)

    def activate_cam(self):
        try:
            if self._current_cam is not None:
                self.clicked_stop()
            elif self._ocu_control_slave.get_access_control() < 2:
                for cam in self._cam_list:
                    if cam.get_cam_button() == self.sender():
                        if not cam.get_state():
                            self._current_cam = cam
                            self.clicked_play()
            else:
                msgBox = QMessageBox()
                msgBox.setText("No access was granted! Use iop_rqt_access_control to get access to the robot!")
                msgBox.exec_()
        except rospy.ServiceException, e:
            print "Can not play the videostream %s" % e

    def control_activated(self, subsystem_name=''):
        return

    def clicked_update(self):
        srvs_name = self._service_update_endpoints
        try:
            update_srvs = rospy.ServiceProxy(srvs_name, QueryByAddr)
            update_srvs(self._ocu_control_slave.get_control_platform())
        except rospy.ServiceException, e:
            print "Service call for update JAUS endpoints failed: %s" % e

    def clicked_play(self):
        try:
            if self._current_cam is None:
                if self._cam_list:
                    self._current_cam = self._cam_list[0]
            if self._current_cam is not None:
                stream_path = self._current_cam.get_endpoint().server_url
                if stream_path:
                    url = QUrl(stream_path.split()[0])
    #       m.aboutToFinish.connect(self.aboutToFinish)
    #       m.bufferStatus.connect(self.aboutToFinish)
    #       m.currentSourceChanged.connect(self.currentSourceChanged)
    #       m.finished.connect(self.finished)
    #       m.hasVideoChanged.connect(self.hasVideoChanged)
    #       m.metaDataChanged.connect(self.metaDataChanged)
    #       m.prefinishMarkReached.connect(self.prefinishMarkReached)
    #       m.seekableChanged.connect(self.seekableChanged)
    #       m.stateChanged.connect(self.stateChanged)
    #       m.tick.connect(self.tick)
    #       m.totalTimeChanged.connect(self.totalTimeChanged)
                    self._media.setCurrentSource(Phonon.MediaSource(url))
                    self.player.play(self._media.currentSource())
                    self._current_cam.set_state(True)
        except rospy.ServiceException, e:
            print "Can not play the video: %s" % e

    def clicked_stop(self):
        try:
            if self._current_cam is not None:
                self._current_cam.set_state(False)
                self._current_cam = None
            self.player.stop()
        except rospy.ServiceException, e:
            print "Can not stoped the video: %s" % e

    def control_platform_callback(self, subsystem_id, node_id, component_id, authority):
        if component_id == 0:
            self.signal_stop.emit()
        else:
            self.signal_update_entpoints.emit()

    def access_control_handler(self, access_control):
        if access_control == OcuControlSlave.ACCESS_CONTROL_RELEASE:
            self.signal_stop.emit()
        else:
            self.signal_play.emit()

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog

        self.dialog_config = QDialog()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_digital_resource_viewer_config.ui')
        loadUi(ui_file, self.dialog_config)

        self.dialog_config.accepted.connect(self.on_dialog_config_accepted)

        # fill configuration dialog
        ti = TopicInfo()
        ti.fill_published_topics(self.dialog_config.comboBox_endpoints_topic, "iop_msgs_fkie/DigitalResourceEndpoints", self._topic_endpoints)  # self._topic_endpoints
        self.dialog_config.comboBox_update_service.clear()
        self.dialog_config.comboBox_update_service.addItem(self._service_update_endpoints)
        default_name = rospy.names.ns_join(rospy.get_namespace(), 'update_endpoints')
        if default_name != self._service_update_endpoints:
            self.dialog_config.comboBox_update_service.addItem(default_name)

    # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def on_dialog_config_accepted(self):
        self.shutdownRosComm()
        self._topic_endpoints = self.dialog_config.comboBox_endpoints_topic.currentText()
        self._service_update_endpoints = self.dialog_config.comboBox_update_service.currentText()
        self.reinitRosComm()
