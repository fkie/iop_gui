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

from std_msgs.msg import UInt16, String
from iop_msgs_fkie.msg import DigitalResourceEndpoints, VisualSensorNames
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
try:
    import vlc
except Exception as e:
    rospy.logwarn("%s\nIf you will use internal vlc player install:\n sudo apt install vlc-nox" % e)
    vlc = None


class DigitalResourceViewer(Plugin):

    signal_topic_endpoints = Signal(DigitalResourceEndpoints)
    signal_topic_names = Signal(VisualSensorNames)
    signal_video_publisher_topic_endpoints = Signal(DigitalResourceEndpoints)

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
        self._publisher_url_list = []
        self._use_multiple_urls = False
        self._use_vlc = True
        self.context = context
        self._topic_video_url = rospy.names.ns_join(rospy.get_namespace(), 'current_video_url')
        self._topic_endpoints = rospy.names.ns_join(rospy.get_namespace(), 'digital_endpoints')
        self._topic_names = rospy.names.ns_join(rospy.get_namespace(), 'visual_sensor_names')
        self._topic_resource_id = rospy.names.ns_join(rospy.get_namespace(), 'dv_resource_id')
        self._subscriber_endpoints = None
        self._subscriber_names = None
        self._publisher_resource_id = None
        self._publisher_current_video_url = None
        self._endpoints = None
        self._resource_names = {}
        self.signal_video_publisher_topic_endpoints.connect(self.create_video_url_publisher)
        self.signal_topic_endpoints.connect(self.signal_callback_endpoints)
        self.signal_topic_names.connect(self.signal_callback_names)
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
        self._rtsp_over_tcp = ''
        self._widget.videoFrame.setVisible(self._use_vlc)
        if self._use_vlc:
            self._create_vlc_player()

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
        instance_settings.set_value('topic_current_video_url', self._topic_video_url)
        instance_settings.set_value('topic_endpoints', self._topic_endpoints)
        instance_settings.set_value('topic_names', self._topic_names)
        instance_settings.set_value('topic_resource_id', self._topic_resource_id)
        instance_settings.set_value('rtps_over_tcp', self._rtsp_over_tcp)
        instance_settings.set_value('use_multi_url', self._use_multiple_urls)
        instance_settings.set_value('use_internal_vlc', self._use_vlc)

    def restore_settings(self, plugin_settings, instance_settings):
        # restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self.shutdownRosComm()
        self._topic_video_url = instance_settings.value('topic_current_video_url', rospy.names.ns_join(rospy.get_namespace(), 'current_video_url'))
        self._topic_endpoints = instance_settings.value('topic_endpoints', rospy.names.ns_join(rospy.get_namespace(), 'digital_endpoints'))
        self._topic_names = instance_settings.value('topic_names', rospy.names.ns_join(rospy.get_namespace(), 'visual_sensor_names'))
        self._topic_resource_id = instance_settings.value('topic_resource_id', rospy.names.ns_join(rospy.get_namespace(), 'dv_resource_id'))
        rtsp_over_tcp = instance_settings.value('rtps_over_tcp', self._rtsp_over_tcp)
        self._use_multiple_urls = instance_settings.value('use_multi_url', self._use_multiple_urls)
        if isinstance(self._use_multiple_urls, (str, unicode)):
            self._use_multiple_urls = self._use_multiple_urls.lower() in ("yes", "true", "t", "1")
        self._use_vlc = instance_settings.value('use_internal_vlc', self._use_vlc)
        if isinstance(self._use_vlc, (str, unicode)):
            self._use_vlc = self._use_vlc.lower() in ("yes", "true", "t", "1")
        if not self._use_multiple_urls:
            self._widget.videoFrame.setVisible(self._use_vlc)
        else:
            self._widget.videoFrame.setVisible(False)  
        if rtsp_over_tcp != self._rtsp_over_tcp:
            self._rtsp_over_tcp = rtsp_over_tcp
            if not self._use_multiple_urls:
                if self._use_vlc:
                    self._create_vlc_player()
            else:
                self._widget.videoFrame.setVisible(False)
        self.reinitRosComm()

    def reinitRosComm(self):
        if self._topic_endpoints:
            self._topic_endpoints = rosgraph.names.script_resolve_name('rostopic', self._topic_endpoints)
            if self._topic_endpoints:
                self._subscriber_endpoints = rospy.Subscriber(self._topic_endpoints, DigitalResourceEndpoints, self.callback_endpoints)
        if self._topic_names:
            self._topic_names = rosgraph.names.script_resolve_name('rostopic', self._topic_names)
            if self._topic_names:
                self._subscriber_names = rospy.Subscriber(self._topic_names, VisualSensorNames, self.callback_names)
        if self._topic_resource_id:
            self._topic_resource_id = rosgraph.names.script_resolve_name('rostopic', self._topic_resource_id)
            if self._topic_resource_id:
                self._publisher_resource_id = rospy.Publisher(self._topic_resource_id, UInt16, latch=True, queue_size=3)
        if self._topic_video_url:
            self._topic_video_url = rosgraph.names.script_resolve_name('rostopic', self._topic_video_url)

    def shutdownRosComm(self):
        if self._subscriber_endpoints:
            self._subscriber_endpoints.unregister()
            self._subscriber_endpoints = None
        if self._subscriber_names:
            self._subscriber_names.unregister()
            self._subscriber_names = None
        if self._publisher_resource_id:
            self._publisher_resource_id.unregister()
            self._publisher_resource_id = None
        if self._publisher_current_video_url:
            self._publisher_current_video_url.unregister()
            self._publisher_current_video_url = None

    def callback_endpoints(self, msg):
        self.signal_topic_endpoints.emit(msg)

    def callback_names(self, msg):
        self.signal_topic_names.emit(msg)
        
    def create_video_url_publisher(self, msg):
        if self._topic_video_url:
            i = 1
            video_url = self._topic_video_url
            for enpoint in msg.endpoints:
                self._publisher_url_list.append([rospy.Publisher(video_url + str(i), String, latch=True, queue_size=3), enpoint.resource_id])
                i += 1

    def signal_callback_endpoints(self, msg):
        # update endpoints
        self._endpoints = msg
        self.signal_video_publisher_topic_endpoints.emit(msg)
        self.create_cam_buttons(msg.endpoints)

    def signal_callback_names(self, msg):
        # update endpoints
        self._resource_names = {}
        for n in msg.names:
            self._resource_names[n.resource_id] = n.name
        if self._resource_names:
            for cam in self._cam_list:
                cam.update_name(self._get_resource_name(cam.get_resource_id()))

    def _get_resource_name(self, resource_id):
        try:
            return self._resource_names[resource_id]
        except Exception:
            return ''

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
            new_cam = Cam(endpoint, self._get_resource_name(endpoint.resource_id))
            if not new_cam.is_in(self._cam_list):
                new_cam.signal_play.connect(self.play)
                new_cam.signal_stop.connect(self.stop)
                self._layout_buttons.addWidget(new_cam)
                self._cam_list.append(new_cam)

    def play(self, url, resource_id):
        if self._use_multiple_urls:
            for pub_topic in self._publisher_url_list:
                if resource_id == pub_topic[1]:
                    pub_topic[0].publish(url)
        else:
            try:
                print "play", url
                self.stop_current()
                if url:
                    ros_msg = UInt16()
                    ros_msg.data = resource_id
                    self._publisher_resource_id.publish(ros_msg)
                    for pub_topic in self._publisher_url_list:
                        if resource_id == pub_topic[1]:
                            pub_topic[0].publish(url)
                    #self._publisher_current_video_url.publish(url)
                    if self._use_vlc:
                        self.media = self.vlc_instance.media_new(url)
            #                self.media.add_option(":network-caching=0")
            #                self.media.add_option(":clock-jitter=0")
            #                self.media.add_option(":clock-synchro=0")
                        # put the media in the media player
                        self.mediaplayer.set_media(self.media)
                        self.mediaplayer.play()
                    self._current_cam = self.sender()
            except rospy.ServiceException, e:
                print "Can not play the video: %s" % e

    def stop(self, url, resource_id):
        if self._use_multiple_urls:
            for pub_topic in self._publisher_url_list:
                if resource_id == pub_topic[1]:
                    pub_topic[0].publish("")
        else:
            if self._current_cam.get_url() == url:
                print "stop", url
                ros_msg = UInt16()
                ros_msg.data = 65535
                self._publisher_resource_id.publish(ros_msg)
                if self._use_vlc:
                    self.mediaplayer.stop()
                self._current_cam = None
                for pub_topic in self._publisher_url_list:
                    if resource_id == pub_topic[1]:
                        pub_topic[0].publish("")
            #self._publisher_current_video_url.publish("")

    def stop_current(self):
        if self._current_cam is not None:
            self._current_cam.set_played(False)
            self.stop(self._current_cam.get_url(),self._current_cam.get_resource_id())
            
    def callback_multi_urls(self):
        if self.dialog_config.checkboxUseMultiUrls.isChecked():
            self.dialog_config.groupBoxUseInternalVlc.setEnabled(True)
      

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
        self.dialog_config = QDialog()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_digital_resource_viewer_config.ui')
        loadUi(ui_file, self.dialog_config)
        self.dialog_config.accepted.connect(self.on_dialog_config_accepted)
        self.dialog_config.groupBoxUseInternalVlc.setChecked(self._use_vlc)
        self.dialog_config.checkboxUseMultiUrls.setChecked(self._use_multiple_urls)
        self.dialog_config.checkboxUseMultiUrls.clicked.connect(self.callback_multi_urls)
        # fill configuration dialog
        ti = TopicInfo()
        ti.fill_subscribed_topics(self.dialog_config.comboBox_video_url_topic, "std_msgs/String", self._topic_video_url)
        ti.fill_published_topics(self.dialog_config.comboBox_endpoints_topic, "iop_msgs_fkie/DigitalResourceEndpoints", self._topic_endpoints)
        ti.fill_published_topics(self.dialog_config.comboBox_name_topic, "iop_msgs_fkie/VisualSensorNames", self._topic_names)
        ti.fill_subscribed_topics(self.dialog_config.comboBox_resource_topic, "std_msgs/UInt16", self._topic_resource_id)
        self.dialog_config.checkBoxRtspOverTcp.setChecked(len(self._rtsp_over_tcp) > 0)

        # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def on_dialog_config_accepted(self):
        self.shutdownRosComm()
        self._topic_video_url = self.dialog_config.comboBox_video_url_topic.currentText()
        self._topic_endpoints = self.dialog_config.comboBox_endpoints_topic.currentText()
        self._topic_names = self.dialog_config.comboBox_name_topic.currentText()
        self._topic_resource_id = self.dialog_config.comboBox_resource_topic.currentText()
        self._use_multiple_urls = self.dialog_config.checkboxUseMultiUrls.isChecked()
        self._use_vlc = self.dialog_config.groupBoxUseInternalVlc.isChecked()
        old_use_vlc = self._use_vlc
        if not self._use_multiple_urls:
            self._widget.videoFrame.setVisible(self._use_vlc)
        else:
            self._widget.videoFrame.setVisible(False)  
        rtsp_over_tcp = ''
        
        if self.dialog_config.checkBoxRtspOverTcp.isChecked():
            rtsp_over_tcp = "--rtsp-tcp"
        if self._rtsp_over_tcp != rtsp_over_tcp or old_use_vlc != self._use_vlc:
            self._rtsp_over_tcp = rtsp_over_tcp
            if not self._use_multiple_urls:
                self._create_vlc_player()
        self.reinitRosComm()

    def _create_vlc_player(self):
        # creating a basic vlc instance
        self.vlc_instance = vlc.Instance(self._rtsp_over_tcp)
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
