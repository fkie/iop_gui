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

from python_qt_binding.QtCore import Qt, Signal

try:
    from python_qt_binding.QtGui import QPushButton
except:
    from python_qt_binding.QtWidgets import QPushButton


class Cam(QPushButton):

    signal_play = Signal(str, int)
    signal_stop = Signal(str)

    def __init__(self, endpoint, parent=None):
        self._endpoint = endpoint
        self.name = "%s - %d" % (Cam.endpoint_type_as_str(endpoint.server_type), endpoint.resource_id)
        QPushButton.__init__(self, self.name, parent)
        self.setCheckable(True)
        self.setFocusPolicy(Qt.NoFocus)
        self.setFixedHeight(24)
#        self.setFixedSize(57, 24)
        self.clicked.connect(self._on_clicked)

    def __repr__(self):
        st = self.endpoint_type_as_str(self._endpoint.server_type)
        return "%s [%s - %d: %d.%d.%d]" % (self._endpoint.server_url, st, self._endpoint.resource_id, self._endpoint.address.subsystem_id, self._endpoint.address.node_id, self._endpoint.address.component_id)

    def _on_clicked(self, checked):
        if checked:
            self.signal_play.emit(self._endpoint.server_url, self._endpoint.resource_id)
            self.setStyleSheet("QPushButton { background-color: #98FB98;}")
        else:
            self.signal_stop.emit(self._endpoint.server_url)
            self.setStyleSheet("QPushButton { background-color: None;}")

    def is_played(self):
        return self.isChecked()

    def set_played(self, state):
        self.setChecked(state)
        if state:
            self.setStyleSheet("QPushButton { background-color: #98FB98;}")
        else:
            self.setStyleSheet("QPushButton { background-color: None;}")

    def get_url(self):
        return self._endpoint.server_url

    def get_endpoint(self):
        return self._endpoint

    def get_subsystem(self):
        return self._point.address.subsystem_id

    def is_endpoint(self, endpoint):
        if self._endpoint.server_type != endpoint.server_type:
            return False
        if self._endpoint.server_url != endpoint.server_url:
            return False
        if self._endpoint.resource_id != endpoint.resource_id:
            return False
        if self._endpoint.address.subsystem_id != endpoint.address.subsystem_id:
            return False
        if self._endpoint.address.node_id != endpoint.address.node_id:
            return False
        if self._endpoint.address.component_id != endpoint.address.component_id:
            return False
        return True

    def is_in(self, endpoints):
        for entpoint in endpoints:
            if isinstance(entpoint, Cam):
                if self.is_endpoint(entpoint.get_endpoint()):
                    return True
            else:
                if self.is_endpoint(entpoint):
                    return True
        return False

    @classmethod
    def endpoint_type_as_str(cls, val):
        if val == 0:
            return 'RTSP'
        elif val == 1:
            return 'MPEG2TS'
        elif val == 2:
            return 'FTP'
        elif val == 3:
            return 'SFTP'
        elif val == 4:
            return 'FTP_SSH'
        elif val == 5:
            return 'HTTP'
        elif val == 6:
            return 'HTTPS'
        elif val == 7:
            return 'SCP'
