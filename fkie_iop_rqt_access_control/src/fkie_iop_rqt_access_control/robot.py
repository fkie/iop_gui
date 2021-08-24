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
from python_qt_binding.QtCore import QObject, Signal, Qt
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QWidget, QDialog, QTreeWidget, QTreeWidgetItem
except:
    from python_qt_binding.QtWidgets import QWidget, QDialog, QTreeWidget, QTreeWidgetItem

import rospy

from .address import Address
from fkie_iop_msgs.msg import OcuCmdEntry, JausAddress
from .handoff_dialog import HandoffDialog


class Robot(QObject):

    MAX_AGE = 120

    control_activated = Signal(Address)
    control_deactivated = Signal(Address)
    view_activated = Signal(Address)
    view_deactivated = Signal(Address)

    def __init__(self, subsystem, settings, authority=205):
        QObject.__init__(self)
        self._subsystem = subsystem
        self._settings = settings
        self._authority = authority
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'robot.ui')
        self._widget = QWidget()
        loadUi(ui_file, self._widget)
        self._last_update = rospy.Time.now()
        self._component_names = dict()
        self._warnings = []
        self._feedback_warnings = dict()
        self._ocu_client = None
        self._warning_dialog = self._create_warning_dialog()
        self._detailed_dialog = self._create_detailed_dialog()
        self.handoff_dialog = HandoffDialog(self.name, self.subsystem_id, self._settings, self._widget)
        self.handoff_dialog.button_blink.connect(self._widget.button_handoff.setEnabled)
        self._widget.button_view.clicked.connect(self._on_robot_view)
        self._widget.button_control.setText("%s - %d" % (subsystem.ident.name, self._subsystem.ident.address.subsystem_id))
        self._widget.button_control.clicked.connect(self._on_robot_control)
        self._widget.button_control.setObjectName(subsystem.ident.name)
        self._widget.button_handoff.setEnabled(False)
        self._widget.button_handoff.clicked.connect(self.on_show_handoff)
        self._widget.button_warnings.setEnabled(False)
        self._widget.button_warnings.clicked.connect(self.on_show_warnings)
        self._widget.button_details.clicked.connect(self.on_show_details)

    def __del__(self):
        self.handoff_dialog.setParent(None)
        self.handoff_dialog.shutdown()
        self.handoff_dialog = None
        self._detailed_dialog = None
        self._warning_dialog = None
        self._ocu_client = None
        self._feedback_warnings.clear()
        self._component_names.clear()
        del self._warnings[:]

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
            self.handoff_dialog.set_client(self._ocu_client)
            self.update_feedback_warnings()
        elif self.has_view() or self.has_control():
            self.set_warnings(["No free OCU client available!", "Start an ocu_client with different nodeID to be able to listen for sensors on second robot."])
            self.handoff_dialog.set_client(None)
        if self._ocu_client is not None:
            self._widget.button_handoff.setVisible(self._ocu_client.has_handoff_publisher())
        else:
            self._widget.button_handoff.setVisible(True)

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
        addr = Address(JausAddress(self._subsystem.ident.address.subsystem_id, 255, 255))
        if checked:
            self._widget.button_view.setChecked(checked)
            self.control_activated.emit(addr)
            self.handoff_dialog.on_access = True
        else:
            self.release_control()
            self.control_deactivated.emit(addr)
            self.handoff_dialog.cancel_handoff()
            self.handoff_dialog.on_access = False
#            if self.has_view():
#                self.view_activated.emit(addr)

    def _on_robot_view(self, checked=False):
        '''
        Click on view robot button. Change to monitor or not controlled state.
        Publishes the signals: view_activated or control_deactivated.
        '''
        addr = Address(JausAddress(self._subsystem.ident.address.subsystem_id, 255, 255))
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
        cmd.authority = self._settings.authority
        cmd.name = self.name
        cmd.address.subsystem_id = self._subsystem.ident.address.subsystem_id
        cmd.address.node_id = 255
        cmd.address.component_id = 255
        if self._widget.button_control.isChecked():
            cmd.access_control = 12
        elif self._widget.button_view.isChecked():
            cmd.access_control = 11
        else:
            cmd.access_control = 10
        if self.ocu_client is not None:
            cmd.ocu_client = self.ocu_client.address
        else:
            cmd.ocu_client.subsystem_id = 65535
            cmd.ocu_client.node_id = 255
            cmd.ocu_client.component_id = 255
        return cmd

    def update(self, subsystem):
        '''
        Applies the updated description of the subsystem.
        :type feedback: fkie_iop_msgs/System
        '''
        if self._subsystem.ident.address.subsystem_id != subsystem.ident.address.subsystem_id:
            return False
#         if self._subsystem.ident.node_id != subsystem.ident.node_id:
#             return False
        if self._subsystem.ident.name != subsystem.ident.name:
            return False
        self._subsystem = subsystem
        # self._last_update = rospy.Time.now()
        return True

    def on_show_handoff(self):
        self.handoff_dialog.setVisible(not self.handoff_dialog.isVisible())

    def on_show_details(self):
        '''
        Shows the subsystem in a new dialog as tree view.
        '''
        twc = self._detailed_dialog.treewidget_components
        twc.clear()
        client_info = "OCU client: ---"
        if self._ocu_client is not None:
            add_info = ''
            if self.ocu_client.subsystem_restricted == self.subsystem_id:
                if self.ocu_client.only_monitor:
                    add_info = ' [restricted, only monitor]'
                else:
                    add_info = ' [restricted]'
            client_info = "OCU client: %s%s" % (self.ocu_client.address, add_info)
        self._detailed_dialog.label_info.setText(client_info)
        if self.name == self._subsystem.ident.name:
            for node in self._subsystem.nodes:
                node_item = QTreeWidgetItem(twc)
                node_name = node.ident.name if node.ident.name else "NODE"
                node_item.setText(0, "%s [id: %d]" % (node_name, node.ident.address.node_id))
                for comp in node.components:
                    cmp_item = QTreeWidgetItem(node_item)
                    cmp_name = self._get_component_name(comp.address)
                    cmp_item.setText(0, "%s [%d.%d.%d]" % (cmp_name, comp.address.subsystem_id, comp.address.node_id, comp.address.component_id))
                    twc.expandItem(node_item)
                    for srv in comp.services:
                        srv_item = QTreeWidgetItem(cmp_item)
                        srv_item.setText(0, "%s v%d.%d" % (srv.uri, srv.major_version, srv.minor_version))
        if self._detailed_dialog.isVisible():
            self._detailed_dialog.setFocus(Qt.ActiveWindowFocusReason)
        else:
            self._detailed_dialog.show()

    def on_show_warnings(self):
        '''
        Shows warning received by feedback.
        '''
        text_browser = self._warning_dialog.warnings
        text_browser.clear()
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
        self._warning_dialog.show()

    def update_feedback_warnings(self):
        '''
        :type warnigns: dict(Address of the ocu client: ServiceInfo)
        '''
        # get all warnings for each subsystem
        warnings = dict()
        if self._ocu_client is not None:
            cw = self._ocu_client.get_warnings(self.subsystem_id, self.has_control())
            warnings.update(cw)
            # get insufficient authority reports to update handoff state button
            insathority = dict()
            cw = self._ocu_client.get_srvs_ins_authority(self.subsystem_id)
            insathority.update(cw)
            # update insufficient authority to activate handoff dialog
            self.handoff_dialog.update_authority_problems(insathority)
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
        self._widget.button_warnings.setEnabled(has_warning)

    def update_ident(self, ident):
        if Address(ident.address) == Address(self._subsystem.ident.address):
            self._last_update = rospy.Time.now()
        if ident.system_type == 60001 or ident.request_type == 4:
            if ident.address.subsystem_id == self._subsystem.ident.address.subsystem_id:
                self._component_names[Address(ident.address)] = ident.name
        return False

    def _get_component_name(self, msg_address):
        addr = Address(msg_address)
        try:
            return self._component_names[addr]
        except Exception:
            pass
        return "Component"

    def is_old(self):
        return rospy.Time.now() - self._last_update > rospy.Duration(self.MAX_AGE)

    def get_widget(self):
        return self._widget

    def _create_warning_dialog(self):
        diag = QDialog(self._widget)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'warning_info.ui')
        loadUi(ui_file, diag)
        diag.resize(600, 250)
        diag.setWindowTitle("Warning for %s[%d]" % (self.name, self.subsystem_id))
        diag.setWindowIcon(QIcon.fromTheme("dialog-warning"))
        return diag

    def _create_detailed_dialog(self):
        diag = QDialog(self._widget)
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'system_info.ui')
        loadUi(ui_file, diag)
        diag.treewidget_components.setHeaderLabel("%s [%d]" % (self.name, self.subsystem_id))
        diag.resize(500, 300)
        diag.setWindowTitle("subsystem %s[%d]" % (self.name, self.subsystem_id))
        diag.setWindowIcon(QIcon.fromTheme("help-about"))
        return diag

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
