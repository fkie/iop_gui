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

from iop_msgs_fkie.msg import System, OcuControlFeedback
from iop_ocu_controllib_fkie import OcuControlMaster
#from os import *
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QRect, QSize, Qt, Signal
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QTreeWidget, QTreeWidgetItem, QPushButton
except:
    from python_qt_binding.QtWidgets import QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QDialog, QTreeWidget, QTreeWidgetItem, QPushButton

from qt_gui.plugin import Plugin
from std_srvs.srv import Empty
import os
import rosgraph
import roslib.message
import rospy

from .robots import Robot
from .topic_info import TopicInfo


class AccessControlClient(Plugin):
    signal_topic = Signal(System)

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
        self.subsystems = None
        self.subsystem_name = None
        self._connection_state = ""
        self._connection_states = ["ACCESS_STATE_NOT_AVAILABLE", "ACCESS_STATE_NOT_CONTROLLED", "ACCESS_STATE_CONTROL_RELEASED", "ACCESS_STATE_CONTROL_ACCEPTED", "ACCESS_STATE_TIMEOUT", "ACCESS_STATE_INSUFFICIENT_AUTHORITY"]
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
        self.vbox = QVBoxLayout()
        self.vbox.setContentsMargins(0, 0, 0, 0)
        self.vbox.setSpacing(1)
        self.vbox.setAlignment(Qt.AlignTop)
        self._widget.groupBox.setLayout(self.vbox)
        self._topic_discovery = rospy.names.ns_join(rospy.get_namespace(), '/iop_system')
        self._service_update_discovery = rospy.names.ns_join(rospy.get_namespace(), '/iop_update_discovery')
        self._subscriber_discovery = None
        self._current_control = ''
        self._system = None
        self._authority = 220
        self.signal_topic.connect(self.signal_callback_subsystem)
        rospy.Subscriber("ocu_control_feedback", OcuControlFeedback, self.ckeched_connection_state)
        self._ocu_control = OcuControlMaster()
        self._ocu_control.set_feedback_handler(self._ocu_control_handler)
        self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
        # rospy.on_shutdown(self.on_ros_shutdown)  # handle the ROS shutdown commands

    def nothing(self, var=None):
        pass

    def create_robot_buttons(self, subsystems):
        subsystems = sorted(subsystems, key=lambda subsystem: subsystem.ident.name)
        subsys = []
        if subsystems is None:
            return
        else:
            i = 0
            if len(self._robotlist) != 0:
                robot_exist = False
                current_robot = []
                for subsystem in subsystems:
                    i = 0
                    robot_exist = False
                    while i < len(self._robotlist):
                        if subsystem.ident.address.subsystem_id == self._robotlist[i].get_robot_subsytem_id() and subsystem.ident.name == self._robotlist[i].get_name():
                            robot_exist = True
                            current_robot.append(self._robotlist[i])
                        i = i + 1
                    if not robot_exist:
                        current_robot.append(self.create_button(subsystem))
                i = 0
                # delete robots
                for rob in self._robotlist:
                    j = 0
                    robot_exist = False
                    while j < len(current_robot):
                        if rob.get_name() == current_robot[j].get_name() and rob.get_robot_subsytem_id() == current_robot[j].get_robot_subsytem_id():
                            robot_exist = True
                        j = j + 1
                    j = 0
                    if not robot_exist:
                        for r in self._robotlist:
                            if r.get_name() == rob.get_name() and r.get_robot_subsytem_id() == rob.get_robot_subsytem_id():
                                j = i
                            i = i + 1
                        self._robotlist.pop(j)
                        widgetToRemove = rob.get_group()
                        self.vbox.removeWidget(widgetToRemove)
                        widgetToRemove.setParent(None)
            elif len(self._robotlist) == 0:
                for subsystem in subsystems:
                    _robot = self.create_button(subsystem)
                self.set_state_buttons_transparent()
        self._widget.show()
        self.subsystems = subsystems
        return subsys

    def create_button(self, subsystem):
        result = None
        try:
            hbox = QHBoxLayout()
            hbox.setContentsMargins(0, 0, 0, 0)
            hbox.setSpacing(1)
            group = QFrame()
            group.setStyleSheet("QFrame { border: None;}")
            group.setMaximumSize(QSize(190, 42))
            hbox.setAlignment(Qt.AlignLeft)

            button0 = QPushButton(subsystem.ident.name)
            button0.setCheckable(True)
            button1 = QPushButton("")
            button2 = QPushButton("")
            button2.setStyleSheet("QPushButton{border: None;background-repeat: no-repeat;}")
            button2.setIcon(QIcon.fromTheme("help-about"))
            button0.setFixedSize(115, 34)
            button1.setFixedSize(25, 25)
            button2.setFixedSize(25, 25)
            button0.clicked.connect(self.clicked_robots)
            button1.clicked.connect(self.show_robot_connection_state)
            button1.setIcon(QIcon.fromTheme("dialog-warning"))
            button1.hide()
            button1.setEnabled(False)
            button1.setStyleSheet("QPushButton{border: None;background-repeat: no-repeat;}")
            button2.clicked.connect(self.show_robot_component)

            hbox.addWidget(button0)
            hbox.addWidget(button2)
            hbox.addWidget(button1)
            group.setLayout(hbox)
            self.vbox.addWidget(group)
            result = Robot(button0, subsystem.ident.address.subsystem_id, button1, button2, group, False, 2, subsystem.ident.name)
            self._robotlist.append(result)
        except rospy.ServiceException, e:
            print "Can not create Buttons: %s" % e
        return result

    def ckeched_connection_state(self, msg):
        for robot in self._robotlist:
            if str(robot.get_robot_subsytem_id()) == str(msg.addr_control.subsystem_id):
                robot.set_connection_msg(msg)
        for robot in self._robotlist:
            if robot.get_connection_msg().access_state == 1 or robot.get_connection_msg().access_state == 4 or robot.get_connection_msg().access_state == 5 or robot.get_connection_msg().access_state == 0:
                robot.get_state_button().show()
                robot.get_state_button().setEnabled(True)
            elif robot.get_connection_msg().access_state == 3 or robot.get_connection_msg().access_state == 2:
                robot.get_state_button().hide()
                robot.get_state_button().setEnabled(False)

    def set_state_buttons_transparent(self):
        for robot in self._robotlist:
            robot.get_state_button().hide()
            robot.get_state_button().setEnabled(False)

    def show_robot_connection_state(self):
        rob_name = ""
        rob_msg = ""
        for robot in self._robotlist:
            if(self.sender() == robot.get_state_button()):
                rob_name = robot.get_name()
                rob_msg = robot.get_connection_msg()
        self._state_window = QDialog(self._widget)
        self.vlayout = QVBoxLayout()
        self.vlayout.setContentsMargins(0, 0, 0, 0)
        self.vlayout.setSpacing(1)
        self.con_state_label = QLabel()
        self.con_state_label.setText(str(rob_msg.addr_reporter.subsystem_id) + "." + str(rob_msg.addr_reporter.node_id) + "." + str(rob_msg.addr_reporter.component_id) + " can not connect to " + str(rob_msg.addr_control.subsystem_id) + "." + str(rob_msg.addr_control.node_id) + "." + str(rob_msg.addr_control.component_id) + ";\n Error: " + str(self._connection_states[rob_msg.access_state]))
        self.vlayout.addWidget(self.con_state_label)
        self._state_window.setWindowTitle(rob_name + "_connection_state")
        self._state_window.setLayout(self.vlayout)
        self._state_window.setWindowIcon(QIcon.fromTheme("dialog-warning"))
        self._state_window.show()

    def show_robot_component(self):
        robot = None
        for robo in self._robotlist:
            if(self.sender() == robo.get_msg_button()):
                robot = robo
        self._componet_window = QDialog(self._widget)
        self.vlayout = QVBoxLayout()
        self.vlayout.setContentsMargins(0, 0, 0, 0)
        self.vlayout.setSpacing(1)
        self.treeWidget_components = QTreeWidget()
        self.treeWidget_components.clear()
        self.treeWidget_components.setGeometry(QRect(10, 50, 275, 180))
        for subsystem in self._system.subsystems:
            if robot.get_name() == subsystem.ident.name:
                addr = subsystem.ident.address
                self.treeWidget_components.clear()
                for node in subsystem.nodes:
                    node_item = QTreeWidgetItem(self.treeWidget_components)
                    node_item.setText(0, "%s [%d.%d.%d]" % (node.ident.name, node.ident.address.subsystem_id, node.ident.address.node_id, node.ident.address.component_id))
                    for cmp in node.components:
                        cmp_item = QTreeWidgetItem(node_item)
                        cmp_item.setText(0, "Component %d.%d.%d" % (cmp.address.subsystem_id, cmp.address.node_id, cmp.address.component_id))
                        self.treeWidget_components.expandItem(node_item)
                        for srv in cmp.services:
                            srv_item = QTreeWidgetItem(cmp_item)
                            srv_item.setText(0, "%s v%d.%d" % (srv.uri, srv.major_version, srv.minor_version))
        self.treeWidget_components.setHeaderLabel("components")
        self._componet_window.resize(300, 250)
        self._componet_window.setWindowTitle("components")
        self.vlayout.addWidget(self.treeWidget_components)
        self._componet_window.setLayout(self.vlayout)
        self._componet_window.setWindowIcon(QIcon.fromTheme("help-about"))
        self._componet_window.show()

    def on_ros_shutdown(self, *args):
        from python_qt_binding.QtGui import QApplication
        QApplication.exit(0)

    def _ocu_control_handler(self, control_feedback):
        # TODO: display the access state of each ocu client
        pass

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._ocu_control.finish()
        self.shutdownRosComm()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        instance_settings.set_value('authority', self._authority)
        instance_settings.set_value('iop_system', self._topic_discovery)
        instance_settings.set_value('iop_update_discovery', self._service_update_discovery)

    def restore_settings(self, plugin_settings, instance_settings):

        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
        self.shutdownRosComm()
        self._authority = (int)(instance_settings.value('authority', 220))
        self._topic_discovery = instance_settings.value('iop_system', rospy.names.ns_join(rospy.get_namespace(), '/iop_system'))
        self._service_update_discovery = instance_settings.value('iop_update_discovery', rospy.names.ns_join(rospy.get_namespace(), '/iop_update_discovery'))
        self.reinitRosComm()

    def reinitRosComm(self):
        if self._topic_discovery:
            self._topic_discovery = rosgraph.names.script_resolve_name('rostopic', self._topic_discovery)
            if self._topic_discovery:
                self._subscriber_discovery = rospy.Subscriber(self._topic_discovery, System, self.callback_system)

    def shutdownRosComm(self):
        if self._subscriber_discovery:
            self._subscriber_discovery.unregister()
            self._subscriber_discovery = None

    def callback_system(self, msg):
        self.signal_topic.emit(msg)

    def signal_callback_subsystem(self, msg):
        self._system = msg
        subsystems = []
        subsystems = self.create_robot_buttons(msg.subsystems)

        # roboter = None
        # for rob in self._robotlist:
        #     if rob.get_name() == self._current_control:
        #         roboter = rob
        self.set_select_robot()

    def control_activated(self, subsystem_name=''):
        self._current_control = subsystem_name

    def clicked_update(self):
        srvs_name = self._service_update_discovery
#    rospy.wait_for_service(srvs_name)
        try:
            update_srvs = rospy.ServiceProxy(srvs_name, Empty)
            update_srvs()
        except rospy.ServiceException, e:
            print "Service call for update JAUS network failed: %s" % e

    def hide_buttons(self):
        for robot in self._robotlist:
            robot.get_msg_button().hide()
            robot.get_state_button().hide()

    def clicked_robots(self):
        _select_robot = None
        for robot in self._robotlist:
            if(self.sender() == robot.get_button()):
                if not robot.get_state():
                    robot.set_status(True)
                    _select_robot = robot
                    self._current_control = _select_robot.get_name()
                    self.set_controled_robot(self._current_control)
                elif robot.get_state():
                    robot.set_status(False)
                    _select_robot = None
                    self._current_control = None
                    self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
            else:
                robot.set_status(False)
        self.set_select_robot()

    def set_controled_robot(self, name):
        if self._system is None or not name:
            return
        if (self._current_control == name):
            for subsystem in self._system.subsystems:
                if name == subsystem.ident.name:
                    addr = subsystem.ident.address
                    self._ocu_control.set_control(addr.subsystem_id, addr.node_id, addr.component_id, (int)(self._authority), OcuControlMaster.ACCESS_CONTROL_REQUEST)
        self._current_control = name

    def set_select_robot(self):
        for robot in self._robotlist:
            if robot.get_state():
                if robot.get_button().isChecked():
                    robot.get_group().setStyleSheet("QFrame { background-color: #ACA5A5;}")
            elif not robot.get_state():
                robot.get_button().setStyleSheet("QFrame { background-color: None;}")
                robot.get_group().setStyleSheet("QFrame { background-color: None;border: None;}")
                robot.get_button().setChecked(False)

#     def _access_state_changed(self, value):
#         if self._current_control == '':
#             self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
#         else:
#             if value == 0:
#                 self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
#             elif value == 1:
#                 self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_ON_DEMAND)
#             elif value == 2:
#                 self._ocu_control.set_access_control(OcuControlMaster.ACCESS_CONTROL_REQUEST)

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
        # stop on cancel pressed
        if not self.dialog_config.exec_():
            return

    def on_dialog_config_accepted(self):
        self.shutdownRosComm()
        self._topic_discovery = self.dialog_config.comboBox_discovery_topic.currentText()
        self._service_update_discovery = self.dialog_config.comboBox_update_service.currentText()
        self._authority = (int)(self.dialog_config.comboBox_authority.currentText())
        self.reinitRosComm()
