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
try:
    from python_qt_binding.QtGui import QWidget
except:
    from python_qt_binding.QtWidgets import QWidget

import rospy

from .address import Address
from fkie_iop_msgs.msg import HandoffResponse


class HandoffRequestWidget(QWidget):
    '''
    A frame with request for a handoff. This widget is used in HandoffDialog to
    visualize and control handoff requests from remote OCU's.
    '''

    MAX_AGE = 60

    STATE_NONE = 0
    STATE_ALLOW = 1
    STATE_WAIT = 2
    STATE_DENY = 3

    response = Signal(HandoffResponse)
    activation_changed = Signal(bool)

    def __init__(self, request):
        QWidget.__init__(self)
        self._request = request
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'handoff_request.ui')
        loadUi(ui_file, self)
        self.button_allow.clicked.connect(self._on_allow)
        self.button_wait.clicked.connect(self._on_wait)
        self.button_deny.clicked.connect(self._on_deny)
        self.ocu_address = Address(request.ocu)
        self.label_requester.setText("Requester: %s" % self.ocu_address)
        self.label_reason.setText("Reason: %s" % request.explanation)
        self._last_update = rospy.Time.now()
        self._current_state = self.STATE_NONE
        self._active = True

    def __hash__(self):
        return hash(self.ocu_address)

    def __eq__(self, other):
        result = self.__hash__() == other.__hash__()
        return result

    @property
    def request_id(self):
        return self._request.request_id

    def set_active(self, state):
        self.setVisible(state)
        changed = self._active != state
        self._active = state
        if (changed):
            self.activation_changed.emit(state)

    def is_active(self):
        return self._active

    def update(self, request):
        # new request -> reset buttons
        if self.request_id != request.request_id:
            self._current_state = self.STATE_NONE
            self.button_allow.setChecked(False)
            self.button_wait.setChecked(False)
            self.button_deny.setChecked(False)
            self.show()
        self._request = request
        self.label_reason.setText("Reason: %s" % request.explanation)
        self._last_update = rospy.Time.now()
        if self._current_state in [self.STATE_NONE, self.STATE_WAIT]:
            self.set_active(True)

    def expired(self):
        return rospy.Time.now() - self._last_update > self.MAX_AGE

    def _on_allow(self, state=False):
        if state and self._current_state in [self.STATE_NONE, self.STATE_WAIT]:
            resp = HandoffResponse()
            resp.code = 0
            resp.request_id = self.request_id
            resp.component = self._request.component
            self.response.emit(resp)
            self.set_active(False)

    def _on_wait(self, state=False):
        if state and self._current_state in [self.STATE_NONE, self.STATE_WAIT]:
            resp = HandoffResponse()
            resp.code = 4
            resp.request_id = self.request_id
            resp.component = self._request.component
            self._current_state = self.STATE_WAIT
            self.response.emit(resp)

    def _on_deny(self, state=False):
        if state and self._current_state in [self.STATE_NONE, self.STATE_WAIT]:
            resp = HandoffResponse()
            resp.code = 3
            resp.request_id = self.request_id
            resp.component = self._request.component
            self._current_state = self.STATE_DENY
            self.response.emit(resp)
            self.set_active(False)
