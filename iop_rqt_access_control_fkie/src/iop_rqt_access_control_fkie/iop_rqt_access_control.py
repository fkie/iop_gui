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

import time
from iop_msgs_fkie.msg import Identification, OcuFeedback, System, JausAddress, OcuCmd

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
try:
    from python_qt_binding.QtGui import QWidget, QDialog
except:
    from python_qt_binding.QtWidgets import QWidget, QDialog

from qt_gui.plugin import Plugin
from std_srvs.srv import Empty
import os
import rosgraph
import rospy

from .client import Client
from .robot import Address, Robot
from .topic_info import TopicInfo


class AccessControlClient(Plugin):
    signal_system = Signal(System)
    signal_feedback = Signal(OcuFeedback)
    signal_ident = Signal(Identification)

    ACCESS_CONTROL_RELEASE = 10
    ACCESS_CONTROL_MONITOR = 11
    ACCESS_CONTROL_REQUEST = 12

    def __init__(self, context):

        # app = QtGui.QApplication.instance()
        # if not app:
        #     app = QtGui.QApplication([])

        super(AccessControlClient, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('AccessControlClient')
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        _args, _unknowns = parser.parse_known_args(context.argv())
        self._widget = QWidget()
        self._robotlist = []
        self._clients = []
        self.subsystems = None
        self.subsystem_name = None
        self._control_client = None  # current control client
        self._view_clients = dict()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        # ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        # loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_access_control.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('AccessControlClientUi')
        self._widget.setWindowTitle("AccessControlClient")
        self._widget.warn_client_button.setVisible(False)
        self._widget.info_frame.setVisible(False)
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
        self._topic_discovery = '/iop_system'
        self._service_update_discovery = '/iop_update_discovery'
        self._topic_identification = '/iop_identification'
        self._topic_cmd = '/ocu_cmd'
        self._topic_feedback = '/ocu_feedback'
        self._authority = 205
        self._access_control = self.ACCESS_CONTROL_RELEASE
        self.signal_system.connect(self.signal_callback_subsystem)
        self.signal_feedback.connect(self.signal_callback_feedback)
        self.signal_ident.connect(self.signal_callback_ident)
        self._pub_cmd = None  # rospy.Publisher(self._topic_cmd, OcuCmd, latch=True, queue_size=10)
        self._sub_feedback = None  # rospy.Subscriber(self._topic_feedback, OcuFeedback, self._ocu_feedback_handler, queue_size=10)
        self._sub_ident = None  # rospy.Subscriber(self._topic_identification, Identification, self._ocu_ident_handler, queue_size=10)
        self._subscriber_discovery = None
        rospy.on_shutdown(self.on_ros_shutdown)
        self._update_timer = rospy.Timer(rospy.Duration(5), self._update_robot_timer)

    def _update_robot_timer(self, event):
        for robot in self._robotlist:
            robot.get_widget().setVisible(not robot.is_old())

    def signal_callback_subsystem(self, msg):
        if not msg.subsystems:
            return
        for subsystem in msg.subsystems:
            updated = False
            for robot in self._robotlist:
                if not updated and robot.update(subsystem):
                    updated = True
            if not updated:
                # add new robot
                robot = Robot(subsystem)
                robot.control_activated.connect(self.on_robot_control_activated)
                robot.control_deactivated.connect(self.on_robot_control_deactivated)
                robot.view_activated.connect(self.on_robot_view_activated)
                robot.view_deactivated.connect(self.on_robot_view_deactivated)
                self._robotlist.append(robot)
                # set an already discovered client if it is restricted to this robot
                for client in self._clients:
                    if robot.subsystem_id == client.subsystem_restricted:
                        if robot.ocu_client is None:
                            robot.ocu_client = client
                # add robot in sort order
                added = False
                for index in range(0, self._widget.layout_robots.count()):
                    if self._widget.layout_robots.itemAt(index).widget().button_control.objectName().lower() > robot.name.lower():
                        self._widget.layout_robots.insertWidget(index, robot.get_widget())
                        added = True
                        break
                if not added:
                    self._widget.layout_robots.addWidget(robot.get_widget())

    def signal_callback_feedback(self, control_feedback):
        '''
        apply feedback to the clients
        '''
        # find the existing client or create a new one
        client = None
        caddr = Address(JausAddress(control_feedback.reporter.subsystem_id, control_feedback.reporter.node_id, 0))
        try:
            index = self._clients.index(caddr)
            client = self._clients[index]
        except ValueError:
            client = Client(caddr.subsystem_id, caddr.node_id)
            self._clients.append(client)
            self._clients.sort()
        client.apply(control_feedback)
        # handle feedback of OCU clients
        for robot in self._robotlist:
            warnings = dict()
            for client in self._clients:
                cw = client.get_warnings(robot.subsystem_id, robot.has_control())
                warnings.update(cw)
                # set client if it is restricted to this robot
                if robot.subsystem_id == client.subsystem_restricted:
                    if robot.ocu_client is None:
                        robot.ocu_client = client
            robot.set_feedback_warnings(warnings)

    def signal_callback_ident(self, ident):
        # update robot alive
        for robot in self._robotlist:
            robot.update_ident(ident)

    def on_robot_control_activated(self, addr):
        '''
        :param addr: Normally only the subsystem ID is set
        :type addr: Address
        '''
        control_ocu = self.get_client_for_control(addr.subsystem_id)
        self._control_client = control_ocu
        deactivated_robot_id = []
        # release current control or view
        for robot in self._robotlist:
            if control_ocu == robot.ocu_client:
                if robot.subsystem_id != addr.subsystem_id:
                    robot.release_control()
                    cmd_release = OcuCmd()
                    cmd_entry1 = robot.state_to_cmd()
                    if robot.ocu_client is not None:
                        if not robot.ocu_client.is_restricted():
                            robot.ocu_client = None
                    cmd_release.cmds.append(cmd_entry1)
                    self._send_cmd(cmd_release)
                    deactivated_robot_id.append(robot.subsystem_id)
            if robot.subsystem_id == addr.subsystem_id:
                if robot.ocu_client is not None:
                    cmd_release = OcuCmd()
                    cmd_entry1 = robot.state_to_cmd()
                    cmd_entry1.access_control = self.ACCESS_CONTROL_RELEASE
                    if not robot.ocu_client.is_restricted():
                        robot.ocu_client = None
                    cmd_release.cmds.append(cmd_entry1)
                    self._send_cmd(cmd_release)
        # set the ocu client for the new robot
        for robot in self._robotlist:
            if robot.subsystem_id == addr.subsystem_id:
                robot.ocu_client = control_ocu
                ocu_str_addr = '---'
                if robot.ocu_client is not None:
                    if not robot.ocu_client.is_restricted():
                        ocu_str_addr = str(robot.ocu_client.address)
                self._widget.label_address.setText(ocu_str_addr)
        # create command for new robot and try to find the view ocu client for deactivated control
        cmd = OcuCmd()
        for robot in self._robotlist:
            if robot.subsystem_id in deactivated_robot_id:
                robot.ocu_client = self.get_client_for_view(deactivated_robot_id)
                if robot.ocu_client is not None:
                    robot.activate_view()
            cmd_entry = robot.state_to_cmd()
            cmd.cmds.append(cmd_entry)
        self._send_cmd(cmd)

    def on_robot_control_deactivated(self, addr):
        '''
        :param addr: address of the robot. Normally only the subsystem ID is set
        :type addr: Address
        '''
        cmd = OcuCmd()
        for robot in self._robotlist:
            cmd_entry = robot.state_to_cmd()
            if robot.subsystem_id == addr.subsystem_id:
                robot.ocu_client = None
                cmd_entry.access_control = self.ACCESS_CONTROL_RELEASE
                self._widget.label_address.setText("---")
            cmd.cmds.append(cmd_entry)
        self._send_cmd(cmd)

    def on_robot_view_activated(self, addr):
        '''
        :param addr: Normally only the subsystem ID is set
        :type addr: Address
        '''
        cmd = OcuCmd()
        ocu_view_client = self.get_client_for_view(addr.subsystem_id)
        for robot in self._robotlist:
            # TODO determine the client for view
            if robot.subsystem_id == addr.subsystem_id:
                robot.ocu_client = ocu_view_client
            cmd_entry = robot.state_to_cmd()
            cmd.cmds.append(cmd_entry)
        self._send_cmd(cmd)

    def on_robot_view_deactivated(self, addr):
        '''
        :param addr: Normally only the subsystem ID is set
        :type addr: Address
        '''
        cmd = OcuCmd()
        for robot in self._robotlist:
            cmd_entry = robot.state_to_cmd()
            if robot.subsystem_id == addr.subsystem_id:
                robot.ocu_client = None
            cmd.cmds.append(cmd_entry)
        self._send_cmd(cmd)

    def get_client_for_control(self, subsystem):
        '''
        Determine for a subsystem a free ocu client to request control.
        :type subsystem: int
        '''
        # do we have already a client assigned
        for robot in self._robotlist:
            if robot.subsystem_id == subsystem:
                result = robot.ocu_client_restricted
                if result is not None:
                    return result
        # look for control client with restricted to the given subsystem
        for client in self._clients:
            ssms = client.subsystem_restricted
            if ssms == subsystem and not client._only_monitor:
                return client
        # take first available
        for client in self._clients:
            ssms = client.subsystem_restricted
            if ssms == 65535 and not client._only_monitor:
                return client
        return None

    def get_client_for_view(self, subsystem):
        '''
        Determine for a subsystem a free ocu client to request monitoring of the sensor data
        :type subsystem: int
        '''
        # already assigned?
        for client in self._clients:
            if client.control_subsystem == subsystem:
                return client
        # look for control client with restricted to the given subsystem
        for client in self._clients:
            ssms = client.subsystem_restricted
            if ssms == subsystem and client.control_subsystem == -1:
                return client
        # take first available
        for client in self._clients:
            ssms = client.subsystem_restricted
            if ssms == 65535 and client.control_subsystem == -1:
                return client
        return None

    def release_all(self):
        cmd = OcuCmd()
        for robot in self._robotlist:
            cmd_entry = robot.state_to_cmd()
            robot.ocu_client = None
            cmd_entry.access_control = self.ACCESS_CONTROL_RELEASE
            self._widget.label_address.setText("---")
            cmd.cmds.append(cmd_entry)
        self._send_cmd(cmd)

    def on_ros_shutdown(self, *args):
        try:
            from python_qt_binding.QtGui import QApplication
            QApplication.exit(0)
        except Exception:
            from python_qt_binding.QtWidgets import QApplication
            QApplication.exit(0)

    def _ocu_feedback_handler(self, control_feedback):
        self.signal_feedback.emit(control_feedback)

    def _ocu_ident_handler(self, ident):
        self.signal_ident.emit(ident)

    def callback_system(self, msg):
        self.signal_system.emit(msg)

    def shutdown_plugin(self):
        # send access release?
        self._update_timer.shutdown()
        self.release_all()
        self.shutdownRosComm()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('authority', self._authority)
        instance_settings.set_value('iop_system', self._topic_discovery)
        instance_settings.set_value('iop_update_discovery', self._service_update_discovery)
        instance_settings.set_value('iop_identification', self._topic_identification)
        instance_settings.set_value('ocu_cmd', self._topic_cmd)
        instance_settings.set_value('ocu_feedback', self._topic_feedback)

    def restore_settings(self, plugin_settings, instance_settings):

        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        # TODO RELEASE ALL CONNROL?
        self.shutdownRosComm()
        self._authority = (int)(instance_settings.value('authority', 220))
        self._topic_discovery = instance_settings.value('iop_system', '/iop_system')
        self._service_update_discovery = instance_settings.value('iop_update_discovery', '/iop_update_discovery')
        self._topic_identification = instance_settings.value('iop_identification', '/iop_identification')
        self._topic_cmd = instance_settings.value('ocu_cmd', '/ocu_cmd')
        self._topic_feedback = instance_settings.value('ocu_feedback', '/ocu_feedback')

        self.reinitRosComm()

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
        self.dialog_config = QDialog()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_access_control_config.ui')
        loadUi(ui_file, self.dialog_config)

        self.dialog_config.accepted.connect(self.on_dialog_config_accepted)

        # fill configuration dialog
        ti = TopicInfo()
        ti.fill_published_topics(self.dialog_config.comboBox_discovery_topic, "iop_msgs_fkie/System", self._topic_discovery)
        default_name = rospy.names.ns_join(rospy.get_namespace(), 'iop_update_discovery')
        if default_name != self._service_update_discovery:
            self.dialog_config.comboBox_update_service.addItem(default_name)
        ti.fill_published_topics(self.dialog_config.comboBox_identification_topic, "iop_msgs_fkie/Identification", self._topic_identification)
        ti.fill_published_topics(self.dialog_config.comboBox_cmd_topic, "iop_msgs_fkie/OcuCmd", self._topic_cmd)
        ti.fill_published_topics(self.dialog_config.comboBox_feedback_topic, "iop_msgs_fkie/OcuFeedback", self._topic_feedback)
        # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def on_dialog_config_accepted(self):
        self.shutdownRosComm()
        self._topic_discovery = self.dialog_config.comboBox_discovery_topic.currentText()
        self._service_update_discovery = self.dialog_config.comboBox_update_service.currentText()
        self._authority = (int)(self.dialog_config.comboBox_authority.currentText())
        self._topic_identification = self.dialog_config.comboBox_identification_topic.currentText()
        self._topic_cmd = self.dialog_config.comboBox_cmd_topic.currentText()
        self._topic_feedback = self.dialog_config.comboBox_feedback_topic.currentText()
        self.reinitRosComm()

    def reinitRosComm(self):
        if self._topic_discovery:
            self._topic_discovery = rosgraph.names.script_resolve_name('rostopic', self._topic_discovery)
            if self._topic_discovery:
                self._subscriber_discovery = rospy.Subscriber(self._topic_discovery, System, self.callback_system)
        if self._pub_cmd is None:
            self._pub_cmd = rospy.Publisher(self._topic_cmd, OcuCmd, latch=True, queue_size=10)
        if self._sub_feedback is None:
            self._sub_feedback = rospy.Subscriber(self._topic_feedback, OcuFeedback, self._ocu_feedback_handler, queue_size=10)
        if self._sub_ident is None:
            self._sub_ident = rospy.Subscriber(self._topic_identification, Identification, self._ocu_ident_handler, queue_size=10)

    def shutdownRosComm(self):
        if self._subscriber_discovery is not None:
            self._subscriber_discovery.unregister()
            self._subscriber_discovery = None
        if self._sub_feedback is not None:
            self._sub_feedback.unregister()
            self._sub_feedback = None
        if self._sub_ident is not None:
            self._sub_ident.unregister()
            self._sub_ident = None
        if self._pub_cmd is not None:
            self._pub_cmd.unregister()
            self._pub_cmd = None

    def clicked_update(self):
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

    def _send_cmd(self, cmd):
        '''
        Publishes the command message to the topic.
        :type cmd: OcuCmd
        '''
        time.sleep(0.1)
        self._pub_cmd.publish(cmd)
