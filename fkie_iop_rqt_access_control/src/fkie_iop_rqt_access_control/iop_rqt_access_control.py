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

import os
import time

from python_qt_binding import loadUi
try:
    from python_qt_binding.QtGui import QWidget
except Exception:
    from python_qt_binding.QtWidgets import QWidget

from qt_gui.plugin import Plugin
import rospy

from fkie_iop_msgs.msg import JausAddress, OcuCmd
from .client import Client
from .robot import Address, Robot
from .settings import Settings


class AccessControlClient(Plugin):
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
        self._access_control = self.ACCESS_CONTROL_RELEASE
        self.settings = Settings()
        self.settings.signal_system.connect(self.signal_callback_subsystem)
        self.settings.signal_feedback.connect(self.signal_callback_feedback)
        self.settings.signal_ident.connect(self.signal_callback_ident)
        rospy.on_shutdown(self.on_ros_shutdown)
        self._update_timer = rospy.Timer(rospy.Duration(5), self._update_robot_timer)

    def _update_robot_timer(self, event):
        for robot in self._robotlist:
            if robot.is_old():
                robot.release_control()
                cmd_release = OcuCmd()
                cmd_entry1 = robot.state_to_cmd()
                cmd_entry1.access_control = self.ACCESS_CONTROL_RELEASE
                if robot.ocu_client is not None and not robot.ocu_client.is_restricted():
                    robot.ocu_client = None
                cmd_release.cmds.append(cmd_entry1)
                self.settings.publish_cmd(cmd_release)
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
                robot = Robot(subsystem, self.settings)
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

    def signal_callback_feedback(self, control_feedback, caller_ns):
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
            client = Client(caddr.subsystem_id, caddr.node_id, caller_ns)
            client.signal_handoff_request.connect(self.signal_handoff_request)
            client.signal_handoff_response.connect(self.signal_handoff_response)
            self._clients.append(client)
            self._clients.sort()
        client.apply(control_feedback)
#         # set client if it is restricted to this robot
#         for client in self._clients:
#             if robot.subsystem_id == client.subsystem_restricted:
#                 if robot.ocu_client is None:
#                     robot.ocu_client = client
        # handle feedback of OCU clients
        for robot in self._robotlist:
            robot.update_feedback_warnings()

    def signal_callback_ident(self, ident):
        # update robot alive
        for robot in self._robotlist:
            robot.update_ident(ident)

    def signal_handoff_request(self, request):
        for robot in self._robotlist:
            if robot.subsystem_id == request.component.subsystem_id:
                robot.handoff_dialog.handle_handoff_request(request)

    def signal_handoff_response(self, response):
        for robot in self._robotlist:
            if robot.subsystem_id == response.component.subsystem_id:
                robot.handoff_dialog.handle_handoff_response(response)

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
                    cmd_entry1.access_control = self.ACCESS_CONTROL_RELEASE
                    if robot.ocu_client is not None:
                        if not robot.ocu_client.is_restricted():
                            robot.ocu_client = None
                    cmd_release.cmds.append(cmd_entry1)
                    self.settings.publish_cmd(cmd_release)
                    deactivated_robot_id.append(robot.subsystem_id)
            if robot.subsystem_id == addr.subsystem_id:
                if robot.ocu_client is not None:
                    cmd_release = OcuCmd()
                    cmd_entry1 = robot.state_to_cmd()
                    cmd_entry1.access_control = self.ACCESS_CONTROL_RELEASE
                    if robot.ocu_client is not None:
                        if not robot.ocu_client.is_restricted():
                            robot.ocu_client = None
                    cmd_release.cmds.append(cmd_entry1)
                    self.settings.publish_cmd(cmd_release)
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
            # if robot.subsystem_id in deactivated_robot_id:
            #     robot.ocu_client = self.get_client_for_view(deactivated_robot_id)
            #     if robot.ocu_client is not None:
            #         robot.activate_view()
            cmd_entry = robot.state_to_cmd()
            cmd.cmds.append(cmd_entry)
        self.settings.publish_cmd(cmd)

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
        self.settings.publish_cmd(cmd)

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
        self.settings.publish_cmd(cmd)

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
        self.settings.publish_cmd(cmd)

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
        self.settings.publish_cmd(cmd)

    def on_ros_shutdown(self, *args):
        try:
            from python_qt_binding.QtGui import QApplication
            QApplication.exit(0)
        except Exception:
            from python_qt_binding.QtWidgets import QApplication
            QApplication.exit(0)

    def shutdown_plugin(self):
        # send access release?
        self._update_timer.shutdown()
        self.release_all()
        del self._robotlist[:]
        self._control_client = None
        for client in self._clients:
            client.shutdown()
        del self._clients[:]
        self.settings.shutdownRosComm()

    def save_settings(self, plugin_settings, instance_settings):
        self.settings.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self.settings.restore_settings(plugin_settings, instance_settings)

    def trigger_configuration(self):
        self.settings.trigger_configuration()

    def on_dialog_config_accepted(self):
        self.settings.on_dialog_config_accepted()

    def clicked_update(self):
        '''
        Currently there are no interface to update the subsystem
        '''
        # send query identification to update the system
        self.settings.update_discovery()
