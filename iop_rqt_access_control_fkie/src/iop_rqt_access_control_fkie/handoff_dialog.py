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
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QIcon
try:
    from python_qt_binding.QtGui import QDialog
except:
    from python_qt_binding.QtWidgets import QDialog

import rospy

from .address import Address
from iop_msgs_fkie.msg import HandoffRequest
from .handoff_request_widget import HandoffRequestWidget


class HandoffDialog(QDialog):

    interaction_needed = Signal(bool)
    button_blink = Signal(bool)

    BLINK_ON_OFF_REL = 4
    BLINK_DURATION = 0.5

    def __init__(self, robot_name, subsystem_id, settings, parent=None):
        QDialog.__init__(self, parent)
        self._settings = settings
        self.name = robot_name
        self.subsystem_id = subsystem_id
        self._handoff_requests = list()
        self._has_insufficient_authority = False
        self._has_requests = False
        self._on_request = False
        self._insufficient_authority_warnings = dict()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'handoff_dialog.ui')
        loadUi(ui_file, self)
        self.resize(400, 450)
        self.setWindowTitle("Handoff for %s[%d]" % (self.name, self.subsystem_id))
        self.setWindowIcon(QIcon.fromTheme("dialog-question"))
        self.lineEdit_explanation.setText(self._settings.handoff_explanation)
        self.checkBox_autorequest.setChecked(self._settings.handoff_autorequest)
        self.button_request_handoff.clicked.connect(self.request_handoff)
        self.button_cancel_handoff.clicked.connect(self.cancel_handoff)
        self.button_close.clicked.connect(self.hide)
        self.on_access = False;
        # emit a signal `button_blink` to emulate the blinking button
        self._blink_timer = rospy.Timer(rospy.Duration(self.BLINK_DURATION), self._update_blink_timer)
        self._blink_last_state = True
        self._blink_count_on = self.BLINK_ON_OFF_REL
        self.frame_own_request.setEnabled(True)

    def __del__(self):
        self._blink_timer.stop()

    def _update_blink_timer(self, event):
        needs = self._has_insufficient_authority or self._has_requests
        state = self._blink_count_on > 0
        if (needs):
            if self.isVisible():
                # do not blink if dialog is open
                state = True
            elif not self.on_access:
                # the request was canceled and we have no active remote requests
                state = False
            if (self._blink_count_on > 0):
                self._blink_count_on -= 1
            else:
                self._blink_count_on = self.BLINK_ON_OFF_REL
        else:
            self._blink_count_on = self.BLINK_ON_OFF_REL
            state = False
        if state != self._blink_last_state:
            self._blink_last_state = state
            self.button_blink.emit(state)

    def request_handoff(self, state=False, info_prefix=''):
        '''
        Request button was clicked. Send a ROS message HandoffRequest.
        '''
        addresses = set()
        for _addr, service_infos in self._insufficient_authority_warnings.items():
            for service_info in service_infos:
                addresses.add(Address(service_info.addr_control))
        for addr in addresses:
            self.label_handoff_state.setText("%srequesting handoff for %s[%d]..." % (info_prefix, self.name, self.subsystem_id))
            cmd = HandoffRequest()
            cmd.request = True
            cmd.authority_code = self._settings.authority
            cmd.explanation = self.lineEdit_explanation.text()
            cmd.component = addr
            self._settings.publish_handoff_request(cmd)
        self._on_request = True

    def cancel_handoff(self):
        '''
        Cancel button was clicked. Send a ROS message HandoffRequest.
        '''
        addresses = set()
        for _addr, service_infos in self._insufficient_authority_warnings.items():
            for service_info in service_infos:
                addresses.add(Address(service_info.addr_control))
        for addr in addresses:
            self.label_handoff_state.setText("request handoff for %s[%d] canceled" % (self.name, self.subsystem_id))
            cmd = HandoffRequest()
            cmd.request = False
            cmd.authority_code = self._settings.authority
            cmd.explanation = self.lineEdit_explanation.text()
            cmd.component = Address(service_info.addr_control)
            self._settings.publish_handoff_request(cmd)
        self._on_request = False

    def handle_handoff_request(self, request):
        hrw = HandoffRequestWidget(request)
        try:
            index = self._handoff_requests.index(hrw)
            if request.request:
                self._handoff_requests[index].set_active(True)
                self._handoff_requests[index].update(request)
            else:
                self._handoff_requests[index].set_active(False)
        except ValueError:
            self._handoff_requests.append(hrw)
            hrw.response.connect(self._on_handoff_own_response)
            hrw.activation_changed.connect(self._update_visible_requests)
            self.requests_layout.addWidget(hrw)
        self._update_visible_requests()
        self.interaction_needed.emit(self._has_insufficient_authority or self._has_requests)

    def _on_handoff_own_response(self, response):
        if response.code != 0:
            pass
        self._settings.publish_handoff_response(response)

    def _update_visible_requests(self, state=False):
        for hrw in self._handoff_requests:
            if hrw.is_active():
                self._has_requests = True
                return True
        self._has_requests = False
        return False

    def update_authority_problems(self, insathority):
        self._insufficient_authority_warnings = insathority
        self._has_insufficient_authority = len(insathority) > 0
        self.frame_own_request.setEnabled(self._has_insufficient_authority)
        if self.checkBox_autorequest.isChecked() and self._has_insufficient_authority:
            self.request_handoff(True, 'auto-')
        self.interaction_needed.emit(self._has_insufficient_authority or self._has_requests)

    def handle_handoff_response(self, response):
        self.label_handoff_state.setVisible(self._has_insufficient_authority)
        self.label_handoff_state.setText("response for %s[%d]: %s" % (self.name, self.subsystem_id, self.handoff_code_to_str(response.code)))

    def handoff_code_to_str(self, code):
        if code == 0:
            return 'GRANTED'
        if code == 1:
            return 'NOT_AVAILABLE'
        if code == 2:
            return 'TIMEOUT'
        if code == 3:
            return 'DENIED'
        if code == 4:
            return 'QUEUED'
        if code == 5:
            return 'DEFERRED'
        if code == 6:
            return 'INSUFFICIENT_AUTHORITY'
        if code == 7:
            return 'WAIT'
        return 'UNKNOWN'
