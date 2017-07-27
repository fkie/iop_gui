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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, QRect, Signal
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QWidget, QVBoxLayout, QLabel, QDialog, QTreeWidget, QTreeWidgetItem, QTextBrowser
except:
    from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QDialog, QTreeWidget, QTreeWidgetItem, QTextBrowser

import rospy

from .address import Address
from iop_msgs_fkie.msg import OcuCmdEntry, JausAddress


class Robot(QObject):

    MAX_AGE = 600

    control_activated = Signal(Address)
    control_deactivated = Signal(Address)
    view_activated = Signal(Address)
    view_deactivated = Signal(Address)

    def __init__(self, subsystem, authority=205):
        QObject.__init__(self)
        self._subsystem = subsystem
        self._authority = authority
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'robot.ui')
        self._widget = QWidget()
        loadUi(ui_file, self._widget)
        self._widget.button_control.setText(subsystem.ident.name)
        self._widget.button_control.clicked.connect(self._on_robot_control)
        self._widget.button_control.setObjectName(subsystem.ident.name)
        self._widget.button_view.clicked.connect(self._on_robot_view)
        self._widget.button_details.clicked.connect(self.on_show_details)
        self._widget.button_warnings.hide()
        self._widget.button_warnings.clicked.connect(self.on_show_warnings)
        self._last_update = rospy.Time.now()
        self._warnings = []
        self._feedback_warnings = dict()
        self._ocu_client = None

    @property
    def name(self):
        return self._subsystem.ident.name

    @property
    def subsystem_id(self):
        # return the subsystem_id of the robot
        return self._subsystem.ident.address.subsystem_id

    @property
    def ocu_client(self):
        return self._ocu_client

    @ocu_client.setter
    def ocu_client(self, ocu_client):
        self.set_warnings([])
        if self._ocu_client is not None:
            self._ocu_client.control_subsystem = -1
        self._ocu_client = ocu_client
        if self._ocu_client is not None:
            self._ocu_client.control_subsystem = self.subsystem_id
            if ocu_client.subsystem_restricted == self.subsystem_id:
                self._widget.button_control.setEnabled(not ocu_client.only_monitor)
        elif self.has_view() or self.has_control():
            self.set_warnings(["No free OCU client available!", "Start an ocu_client with different nodeID to be able to listen for sensors on second robot."])

    @property
    def ocu_client_restricted(self):
        if self._ocu_client is not None:
            if self._ocu_client.subsystem_restricted == self.subsystem_id:
                return self._ocu_client
        return None

    def set_control_active(self, state):
        self._widget.button_control.setEnabled(state)

    def _on_robot_control(self, checked=False):
        '''
        Click on control robot button. Change to controlled or monitor state.
        Publishes the signals: control_activated or view_activated.
        '''
        addr = Address(JausAddress(self._subsystem.ident.address.subsystem_id, 0, 0))
        if checked:
            self._widget.button_view.setChecked(checked)
            self.control_activated.emit(addr)
        else:
            self.release_control()
            self.control_deactivated.emit(addr)
#            if self.has_view():
#                self.view_activated.emit(addr)

    def _on_robot_view(self, checked=False):
        '''
        Click on view robot button. Change to monitor or not controlled state.
        Publishes the signals: view_activated or control_deactivated.
        '''
        addr = Address(JausAddress(self._subsystem.ident.address.subsystem_id, 0, 0))
        if checked:
            self._widget.button_view.setChecked(checked)
            self.view_activated.emit(addr)
        else:
            if self.has_control():
                self._widget.button_control.setChecked(False)
                self.control_deactivated.emit(addr)
            self.view_deactivated.emit(addr)

    def has_control(self):
        return self._widget.button_control.isChecked()

    def has_view(self):
        return self._widget.button_view.isChecked()

    def release_control(self):
        self._widget.button_view.setChecked(False)
        self._widget.button_control.setChecked(False)

    def activate_view(self):
        self._widget.button_view.setChecked(True)

    def state_to_cmd(self):
        cmd = OcuCmdEntry()
        cmd.authority = self._authority
        cmd.name = self.name
        cmd.address.subsystem_id = self._subsystem.ident.address.subsystem_id
        if self._widget.button_control.isChecked():
            cmd.access_control = 12
        elif self._widget.button_view.isChecked():
            cmd.access_control = 11
        else:
            cmd.access_control = 10
        if self.ocu_client is not None:
            cmd.ocu_client = self.ocu_client.address
        return cmd

    def update(self, subsystem):
        '''
        Applies the updated description of the subsystem.
        :type feedback: iop_msgs_fkie/System
        '''
        if self._subsystem.ident.address.subsystem_id != subsystem.ident.address.subsystem_id:
            return False
#         if self._subsystem.ident.node_id != subsystem.ident.node_id:
#             return False
        if self._subsystem.ident.name != subsystem.ident.name:
            return False
        self._subsystem = subsystem
        self._last_update = rospy.Time.now()
        return True

    def on_show_details(self):
        '''
        Shows the subsystem in a new dialog as tree view.
        '''
        diag = QDialog(self._widget)
        vlayout = QVBoxLayout()
        vlayout.setContentsMargins(0, 0, 0, 0)
        vlayout.setSpacing(1)
        treeWidget_components = QTreeWidget()
        treeWidget_components.clear()
        treeWidget_components.setGeometry(QRect(10, 50, 275, 180))
        if self._ocu_client is not None:
            add_info = ''
            if self.ocu_client.subsystem_restricted == self.subsystem_id:
                if self.ocu_client.only_monitor:
                    add_info = ' [restricted, only monitor]'
                else:
                    add_info = ' [restricted]'
            vlayout.addWidget(QLabel("OCU client: %s%s" % (self.ocu_client.address, add_info)))
        if self.name == self._subsystem.ident.name:
            treeWidget_components.clear()
            for node in self._subsystem.nodes:
                node_item = QTreeWidgetItem(treeWidget_components)
                node_item.setText(0, "%s [%d.%d.%d]" % (node.ident.name, node.ident.address.subsystem_id, node.ident.address.node_id, node.ident.address.component_id))
                for comp in node.components:
                    cmp_item = QTreeWidgetItem(node_item)
                    cmp_item.setText(0, "Component %d.%d.%d" % (comp.address.subsystem_id, comp.address.node_id, comp.address.component_id))
                    treeWidget_components.expandItem(node_item)
                    for srv in comp.services:
                        srv_item = QTreeWidgetItem(cmp_item)
                        srv_item.setText(0, "%s v%d.%d" % (srv.uri, srv.major_version, srv.minor_version))
        treeWidget_components.setHeaderLabel("%s [%d]" % (self.name, self.subsystem_id))
        diag.resize(300, 250)
        diag.setWindowTitle("subsystem [%d]" % self.subsystem_id)
        vlayout.addWidget(treeWidget_components)
        diag.setLayout(vlayout)
        diag.setWindowIcon(QIcon.fromTheme("help-about"))
        diag.show()

    def on_show_warnings(self):
        '''
        Shows warning received by feedback.
        '''
        diag = QDialog(self._widget)
        vlayout = QVBoxLayout()
        vlayout.setContentsMargins(0, 0, 0, 0)
        vlayout.setSpacing(1)
        text_browser = QTextBrowser()
        vlayout.addWidget(text_browser)
        if not self._warnings and not self._feedback_warnings:
            text_browser.append('No known warnings!')
        else:
            for msg in self._warnings:
                text_browser.append(msg)
            if self._feedback_warnings:
                text_browser.append('Services with warning state:')
                for client, service_infos in self._feedback_warnings.items():
                    text_browser.append("Client %s:" % client)
                    for service_info in service_infos:
                        text_browser.append("    %s[%s]: %s" % (service_info.uri, Address(service_info.addr_control), self.access_state_to_str(service_info.access_state)))
        diag.setWindowTitle("Warning for %s" % self.name)
        diag.setLayout(vlayout)
        diag.setWindowIcon(QIcon.fromTheme("dialog-warning"))
        diag.show()

    def set_feedback_warnings(self, warnings):
        '''
        :type warnigns: dict(Address of the ocu client: ServiceInfo)
        '''
        self._feedback_warnings = warnings
        self._update_warnings_button()

    def set_warnings(self, warnings):
        '''
        :type warnigns: list of strings
        '''
        self._warnings = warnings
        self._update_warnings_button()

    def _update_warnings_button(self):
        has_warning = (len(self._warnings) + len(self._feedback_warnings)) > 0
        if has_warning and self.has_control():
            self._widget.button_control.setStyleSheet("QPushButton { background-color: #FE9A2E;}")
        elif self.has_control():
            self._widget.button_control.setStyleSheet("QPushButton { background-color: #98FB98;}")
            self._widget.button_view.setStyleSheet("QPushButton { background-color: #98FB98;}")
        elif self.has_view():
            self._widget.button_control.setStyleSheet("QPushButton { background-color: None;}")
            self._widget.button_view.setStyleSheet("QPushButton { background-color: #98FB98;}")
        else:
            self._widget.button_control.setStyleSheet("QPushButton { background-color: None;}")
            self._widget.button_view.setStyleSheet("QPushButton { background-color: None;}")
        self._widget.button_warnings.setVisible(has_warning)

    def update_ident(self, ident):
        if Address(ident.address) == Address(self._subsystem.ident.address):
            self._last_update = rospy.Time.now()

    def is_old(self):
        return rospy.Time.now() - self._last_update > self.MAX_AGE

    def get_widget(self):
        return self._widget

    def access_state_to_str(self, state):
        if state == 0:
            return 'NOT_AVAILABLE'
        if state == 1:
            return 'NOT_CONTROLLED'
        if state == 2:
            return 'CONTROL_RELEASED'
        if state == 3:
            return 'CONTROL_ACCEPTED'
        if state == 4:
            return 'TIMEOUT'
        if state == 5:
            return 'INSUFFICIENT_AUTHORITY'
        if state == 6:
            return 'MONITORING'
        return 'UNKNOWN'
