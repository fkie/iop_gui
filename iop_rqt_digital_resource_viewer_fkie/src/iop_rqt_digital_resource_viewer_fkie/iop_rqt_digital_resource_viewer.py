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
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Qt
from python_qt_binding.QtGui import QIcon, QPalette, QColor
try:
    from python_qt_binding.QtGui import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QMessageBox
except:
    from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QMessageBox

from qt_gui.plugin import Plugin
import os
import sys
import rosgraph
import rospy

from .cam import Cam
from .topic_info import TopicInfo
import vlc


class DigitalResourceViewer(Plugin):

    signal_topic_endpoints = Signal(DigitalResourceEndpoints)

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
        self._topic_endpoints = rospy.names.ns_join(rospy.get_namespace(), 'digital_endpoints')
        self._subscriber_endpoints = None
        self._endpoints = None
        self.signal_topic_endpoints.connect(self.signal_callback_endpoints)
        self._widget.pushButton_info.clicked.connect(self.show_info)
        # self._widget.pushButton_info.setIcon(QIcon.fromTheme("help-about"))
        self._widget.pushButton_info.setStyleSheet("QPushButton{border: None;background-repeat: no-repeat;}")
        self._layout_buttons = QHBoxLayout()
        self._widget.buttonsFrame.setLayout(self._layout_buttons)
        self._current_cam = None

        # create video frame
        self.palette = self._widget.videoFrame.palette()
        self.palette.setColor(QPalette.Window, QColor(0, 0, 0))
        self._widget.videoFrame.setPalette(self.palette)
        self._widget.videoFrame.setAutoFillBackground(True)
        # creating a basic vlc instance
        self.vlc_instance = vlc.Instance()
        # creating an empty vlc media player
        self.mediaplayer = self.vlc_instance.media_player_new()
        # the media player has to be 'connected' to the QFrame
        # (otherwise a video would be displayed in it's own window)
        # this is platform specific!
        # you have to give the id of the QFrame (or similar object) to
        # vlc, different platforms have different functions for this
        if sys.platform.startswith('linux'):  # for Linux using the X Server
            self.mediaplayer.set_xwindow(self._widget.videoFrame.winId())
        elif sys.platform == "win32":  # for Windows
            self.mediaplayer.set_hwnd(self._widget.videoFrame.winId())
        elif sys.platform == "darwin":  # for MacOS
            self.mediaplayer.set_nsobject(int(self._widget.videoFrame.winId()))

        self.media = self.vlc_instance.media_new("https://youtu.be/h4rhdZ_MXf8?t=17")
        # put the media in the media player
        self.mediaplayer.set_media(self.media)
        self.mediaplayer.play()

    def show_info(self):
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
        # unregister all publishers here
        self.stop_current()
        self.shutdownRosComm()

    def save_settings(self, plugin_settings, instance_settings):
        # save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('topic_endpoints', self._topic_endpoints)

    def restore_settings(self, plugin_settings, instance_settings):
        # restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self.shutdownRosComm()
        self._topic_endpoints = instance_settings.value('topic_endpoints', rospy.names.ns_join(rospy.get_namespace(), 'digital_endpoints'))
        self.reinitRosComm()

    def reinitRosComm(self):
        if self._topic_endpoints:
            self._topic_endpoints = rosgraph.names.script_resolve_name('rostopic', self._topic_endpoints)
            if self._topic_endpoints:
                self._subscriber_endpoints = rospy.Subscriber(self._topic_endpoints, DigitalResourceEndpoints, self.callback_endpoints)

    def shutdownRosComm(self):
        if self._subscriber_endpoints:
            self._subscriber_endpoints.unregister()
            self._subscriber_endpoints = None

    def callback_endpoints(self, msg):
        self.signal_topic_endpoints.emit(msg)

    def signal_callback_endpoints(self, msg):
        # update endpoints
        self._endpoints = msg
        self.create_cam_buttons(msg.endpoints)

    def create_cam_buttons(self, endpoints):
        if self._cam_list:
            new_cam_list = []
            for cam in self._cam_list:
                if not cam.is_in(endpoints):
                    self._layout_buttons.removeWidget(cam)
                    cam.setParent(None)
                else:
                    new_cam_list.append(cam)
            self._cam_list = new_cam_list

        for endpoint in endpoints:
            new_cam = Cam(endpoint)
            if not new_cam.is_in(self._cam_list):
                new_cam.signal_play.connect(self.play)
                new_cam.signal_stop.connect(self.stop)
                self._layout_buttons.addWidget(new_cam)
                self._cam_list.append(new_cam)

    def play(self, url):
        try:
            print "play", url
            self.stop_current()
            if url:
                self.media = self.vlc_instance.media_new(url)
                # put the media in the media player
                self.mediaplayer.set_media(self.media)
                self.mediaplayer.play()
                self._current_cam = self.sender()
        except rospy.ServiceException, e:
            print "Can not play the video: %s" % e

    def stop(self, url):
        print "stop", url
        if self._current_cam.get_url() == url:
            self.mediaplayer.stop()
            self._current_cam = None

    def stop_current(self):
        if self._current_cam is not None:
            self._current_cam.set_played(False)
            self.stop(self._current_cam.get_url())

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
        # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def on_dialog_config_accepted(self):
        self.shutdownRosComm()
        self._topic_endpoints = self.dialog_config.comboBox_endpoints_topic.currentText()
        self.reinitRosComm()
