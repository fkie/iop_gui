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


from python_qt_binding import loadUi
from python_qt_binding.QtCore import QObject, Signal, QTimer
from python_qt_binding.QtGui import QBrush, QColor, QPalette
try:
    from python_qt_binding.QtGui import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSpacerItem, QSizePolicy, QDialog, QTableWidgetItem, QFrame, QPushButton, QSizePolicy
except:
    from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSpacerItem, QSizePolicy, QDialog, QTableWidgetItem, QFrame, QPushButton, QSizePolicy
import os

from qt_gui.plugin import Plugin
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
import rosgraph
import roslib.message
import rospy

from .topic_info import TopicInfo


class JointStateGroup(QFrame):
  def __init__(self, caller, jointstate, parent=None, stored_topic=None):
    super(JointStateGroup, self).__init__(parent)
    ti = TopicInfo()
    # create a new group
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_velocity_joint_group.ui')
    loadUi(ui_file, self)
    group_layout = QVBoxLayout(self.frame)
    group_layout.setContentsMargins(0, 0, 0, 0)
    group_layout.setSpacing(0)
    self._cmd_type = Float64MultiArray
    if len(jointstate.name) == 1:
      self._cmd_type = Float64
    ti.fill_subscribed_topics(self.comboBox_cmdTopic, self._cmd_type._type, stored_topic)
    for joint_name in jointstate.name:
      joint_frame = QFrame()
      ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_velocity_joint.ui')
      loadUi(ui_file, joint_frame)
      joint_frame.setObjectName(joint_name)
      joint_frame.labelJointName.setText(joint_name)
      joint_frame.lineEditCurrentValue.setText('0.0')
      joint_frame.doubleSpinBoxNewValue.setValue(0.0)
      joint_frame.layout().setContentsMargins(0, 0, 0, 0)
      joint_frame.layout().setSpacing(0)
      group_layout.addWidget(joint_frame)
#    self._widget.layout().insertWidget(self._widget.layout().count() - 1, group_frame)
#    self._jointgroups[caller] = group_frame
    self.setObjectName(caller)
    self._caller = caller
    self._jointstate = jointstate
    self._topic_command = self.comboBox_cmdTopic.currentText()
    self.pushButtonSend.clicked.connect(self.on_clicked_send)
    self.pushButtonSendZero.clicked.connect(self.on_clicked_send_zero)
    self.comboBox_cmdTopic.activated.connect(self.on_activated_topic)
    self._cmd_publisher = None
    if self._cmd_type and self._topic_command:
      self._cmd_publisher = rospy.Publisher(self._topic_command, self._cmd_type, queue_size=1)

  def shutdown_roscomm(self):
    if self._cmd_publisher is not None:
      self._cmd_publisher.unregister()
      self._cmd_publisher = None

  def clear_parent(self):
    self.setParent( None )

  def on_clicked_send(self):
    cmd_vals = self._get_value_array()
    msg = self._cmd_type()
    if isinstance(msg, Float64MultiArray):
      for val in cmd_vals:
        msg.data.append(val)
    elif isinstance(msg, Float64):
      for val in cmd_vals:
        msg.data = val
        break
    if self._cmd_publisher is not None:
      self._cmd_publisher.publish(msg)

  def on_clicked_send_zero(self):
    msg = self._cmd_type()
    if isinstance(msg, Float64MultiArray):
      cmd_vals = self._get_value_array()
      for val in cmd_vals:
        msg.data.append(0.)
    elif isinstance(msg, Float64):
      msg.data = 0.
    if self._cmd_publisher is not None:
      self._cmd_publisher.publish(msg)

  def on_activated_topic(self, arg):
    self._topic_command = self.comboBox_cmdTopic.itemText(arg)
    self.shutdown_roscomm()
    if self._cmd_type and self._topic_command:
      self._cmd_publisher = rospy.Publisher(self._topic_command, self._cmd_type, queue_size=1)

  def update(self, jointstate):
    for index_name in range(len(jointstate.name)):
      joint_name = jointstate.name[index_name]
      for j in range(self.frame.layout().count()):
        joint_widget = self.frame.layout().itemAt(j).widget()
        if joint_widget.labelJointName.text() == joint_name:
          if index_name < len(jointstate.velocity):
            joint_widget.lineEditCurrentValue.setText("%.2f" % jointstate.velocity[index_name])

  def get_command_topic(self):
    return self._topic_command

  def _get_value_array(self):
    result = []
    for j in range(self.frame.layout().count()):
      result.append(self.frame.layout().itemAt(j).widget().doubleSpinBoxNewValue.value())
    return result


class VelocityControl(Plugin):

  signal_topic = Signal(JointState)

  def __init__(self, context):
    super(VelocityControl, self).__init__(context)
    # Give QObjects reasonable names
    self.setObjectName('VelocityControl')

    # Process standalone plugin command-line arguments
    from argparse import ArgumentParser
    parser = ArgumentParser()
    # Add argument(s) to the parser.
    parser.add_argument("-q", "--quiet", action="store_true",
            dest="quiet",
            help="Put plugin in silent mode")
    args, unknowns = parser.parse_known_args(context.argv())

    # Create QWidget
    self._widget = QWidget()
    # Get path to UI file which is a sibling of this file
    # in this example the .ui and .py file are in the same folder
    #ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
    # Extend the widget with all attributes and children from UI file
    #loadUi(ui_file, self._widget)
    # Give QObjects reasonable names
    self._widget.setObjectName('VelocityControlUi')
    vLayout = QVBoxLayout(self._widget)
    vLayout.setContentsMargins(0, 0, 0, 0)
    vLayout.setSpacing(0)
    self._widget.setLayout(vLayout)
    self._widget.layout().setSpacing(0)
    self._widget.setWindowTitle("VelocityControl");
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
    self._topic_list = ''
    self._topic_commands = {}
    self._jointgroups = {}
    self.signal_topic.connect( self.signal_callback_list )

    rospy.on_shutdown(self.on_ros_shutdown) #handle the ROS shutdown commands

  def on_ros_shutdown(self, *args):
    from python_qt_binding.QtGui import QApplication
    QApplication.exit(0)

  def shutdown_plugin(self):
    # TODO unregister all publishers here
    self.shutdownRosComm()

  def save_settings(self, plugin_settings, instance_settings):
    # TODO save intrinsic configuration, usually using:
    # instance_settings.set_value(k, v)
    self._topic_commands.clear()
    for caller, joint_frame in self._jointgroups.items():
      self._topic_commands[caller] = joint_frame.get_command_topic()
    instance_settings.set_value('topic_commands', self._topic_commands)
    instance_settings.set_value('topic_list', self._topic_list)

  def restore_settings(self, plugin_settings, instance_settings):
    # TODO restore intrinsic configuration, usually using:
    # v = instance_settings.value(k)

    self.shutdownRosComm()

    self._topic_commands = instance_settings.value('topic_commands', {})
    self._topic_list = instance_settings.value('topic_list', 'joint_state')

    self.fill_widget()
    self.reinitRosComm()

  def reinitRosComm(self):
#     if self._topic_command:
#       self._topic_command = rosgraph.names.script_resolve_name('rostopic', self._topic_command)
#       if (self._topic_command):
#         self._publisher_command = rospy.Publisher(self._topic_command, PowerSwitch, queue_size=1)
    if self._topic_list:
      self._topic_list = rosgraph.names.script_resolve_name('rostopic', self._topic_list)
      if self._topic_list:
        self._subscriber_list = rospy.Subscriber(self._topic_list, JointState, self.callback_jointstate)

  def clearstuff(self):
    pass
#    for name,p in self._devices.items():
#      self._widget.layout().removeWidget(p)
#      p.clearParent()
#    self._devices.clear();

  def shutdownRosComm(self):
    self.clearstuff()
#    if self._topic_command:
#      self._publisher_command.unregister()
#      self._publisher_command = None
    if self._topic_list:
      self._subscriber_list.unregister()
      self._subscriber_list = None

  def fill_widget(self):
#    for i in range(len(self._topics)):
    self._widget.layout().addItem(QSpacerItem(1,1,QSizePolicy.Expanding, QSizePolicy.Expanding))

  def callback_jointstate(self, msg):
    self.signal_topic.emit(msg)

  def signal_callback_list( self, msg ):
    # update
    caller = msg._connection_header['callerid']
    if caller not in self._jointgroups:
      stored_topic = None
      if caller in self._topic_commands:
        stored_topic = self._topic_commands[caller]
      group_frame = JointStateGroup(caller, msg, stored_topic=stored_topic)
      self._widget.layout().insertWidget(self._widget.layout().count() - 1, group_frame)
      self._jointgroups[caller] = group_frame
    else:
      # update the values
      self._jointgroups[caller].update(msg)

  def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure it
    # Usually used to open a configuration dialog

    self.dialog_config = QDialog()
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iop_rqt_velocity_config.ui')
    loadUi(ui_file, self.dialog_config)

    self.dialog_config.accepted.connect(self.on_dialog_config_accepted)

    # fill configuration dialog
    ti = TopicInfo()
    ti.fill_published_topics(self.dialog_config.comboBox_listTopic, "sensor_msgs/JointState", self._topic_list)
#    ti.fill_subscribed_topics(self.dialog_config.comboBox_commandTopic, "std_msgs/Float64MultiArray", self._topic_command)

  # stop on cancel pressed
    if not self.dialog_config.exec_():
       return

  def on_dialog_config_accepted(self):
    self.shutdownRosComm()
#    self._topic_command = self.dialog_config.comboBox_commandTopic.currentText()
    self._topic_list = self.dialog_config.comboBox_listTopic.currentText()
    self.reinitRosComm()
