// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Author: Alexander Tiderko
// Copyright 2022 Fraunhofer FKIE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Based on the mapviz plugin of
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// *****************************************************************************

#include <fkie_iop_mapviz_plugins/access_control_robot.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QIcon>
#include <QTextBrowser>
#include <QTreeWidgetItem>

// ROS libraries
#include <ros/master.h>



namespace fkie_iop_mapviz_plugins
{
  AccessControlRobot::AccessControlRobot(fkie_iop_msgs::Subsystem subsystem, int authority) :
    QWidget(),
    ocu_client_(NULL),
    subsystem_(subsystem),
    authority_(authority),
    last_update_(ros::WallTime::now()),
    detailed_dialog_(new QDialog()),
    warning_dialog_(new QDialog())
  {
    ui_robot_.setupUi(this);
    std::string robotname = subsystem_.ident.name;
    std::string buttonname = robotname + " - " + std::to_string(getSubsystemID());
    // setup system info dialog
    ui_system_info_.setupUi(detailed_dialog_);
    std::string header_str = robotname + "[" + std::to_string(getSubsystemID()) + "]";
    ui_system_info_.treewidget_components->setHeaderLabel(header_str.c_str());
    detailed_dialog_->resize(500, 300);
    std::string title_str = "subsystem " + robotname + "[" + std::to_string(getSubsystemID()) + "]";
    detailed_dialog_->setWindowTitle(title_str.c_str());
    detailed_dialog_->setWindowIcon(QIcon::fromTheme("help-about"));
    // setup warning info dialog
    ui_warning_info_.setupUi(warning_dialog_);
    // setup buttons
    ui_robot_.button_control->setText(buttonname.c_str());
    ui_robot_.button_control->setObjectName(subsystem_.ident.name.c_str());
    ui_robot_.button_handoff->setEnabled(false);
    ui_robot_.button_warnings->setEnabled(false);
    // connect signals
    QObject::connect(ui_robot_.button_view, SIGNAL(clicked(bool)), this, SLOT(onRobotView(bool)));
    QObject::connect(ui_robot_.button_control, SIGNAL(clicked(bool)), this, SLOT(onRobotControl(bool)));
    QObject::connect(ui_robot_.button_handoff, SIGNAL(clicked(bool)), this, SLOT(onShowHandoff(bool)));
    QObject::connect(ui_robot_.button_warnings, SIGNAL(clicked(bool)), this, SLOT(onShowWarnings(bool)));
    QObject::connect(ui_robot_.button_details, SIGNAL(clicked(bool)), this, SLOT(onShowDetails(bool)));
    setVisible(false);
  }

  AccessControlRobot::~AccessControlRobot()
  {
    ROS_INFO("delete access control robot %s", subsystem_.ident.name.c_str());
  }

  /**
   * @brief: applies the updated description of the subsystem.
   * @param subsystem fkie_iop_msgs/Subsystem:
   **/
  bool AccessControlRobot::updateSubsystem(const fkie_iop_msgs::Subsystem& subsystem)
  {
    if (getSubsystemID() != subsystem.ident.address.subsystem_id) {
      return false;
    }
    if (name() != subsystem.ident.name) {
      return false;
    }
    subsystem_ = subsystem;
    // self._last_update = rospy.Time.now()
    return true;
  }

  Ui::Robot& AccessControlRobot::getRobotUI()
  {
    return ui_robot_;
  }

  std::string AccessControlRobot::name()
  {
    return subsystem_.ident.name;
  }

  int AccessControlRobot::getSubsystemID()
  {
    return subsystem_.ident.address.subsystem_id;
  }


    // @property
    // def ocuClientRestricted(self):
    //     if self._ocu_client is not None:
    //         if self._ocu_client.subsystem_restricted == self.subsystem_id:
    //             return self._ocu_client
    //     return None

  AccessControlClient* AccessControlRobot::getOcuClient()
  {
    return ocu_client_;
  }

  void AccessControlRobot::setOcuClient(AccessControlClient* client)
  {
    warnings_.clear();
    updateWarningsButton();
    if (ocu_client_ != NULL) {
        ocu_client_->control_subsystem = -1;
    }
    ocu_client_ = client;
    if (ocu_client_ != NULL) {
        ocu_client_->control_subsystem = getSubsystemID();
        if (ocu_client_->restrictedToSubsystem() == getSubsystemID()) {
            ui_robot_.button_control->setEnabled(!ocu_client_->onlyMonitor());
        }
        // handoff_dialog_.set_client(ocu_client_);
        updateFeedbackWarnings();
    } else if (hasView() || hasControl()) {
        std::vector<std::string> warnings;
        warnings.push_back("No free OCU client available!");
        warnings.push_back("Start an ocu_client with different nodeID to be able to listen for sensors on second robot.");
        setWarnings(warnings);
        //handoff_dialog.set_client(NULL);
    } else {
      updateWarningsButton();
    }
    if (ocu_client_ != NULL) {
      ui_robot_.button_handoff->setVisible(ocu_client_->hasHandoffPublisher());
      setControlAddr(ocu_client_->address());
    } else {
      setControlAddr(JausAddress());
      ui_robot_.button_handoff->setVisible(true);
    }
  }

  JausAddress AccessControlRobot::getControlAddr()
  {
    return control_addr_;
  }

  void AccessControlRobot::setControlAddr(JausAddress address)
  {
    control_addr_ = address;
    //_updateWarningsButton();
  }

  void AccessControlRobot::setControlActive(bool state)
  {
    ui_robot_.button_control->setEnabled(state);
  }

  void AccessControlRobot::onRobotView(bool checked)
  {
    ROS_INFO("IOP access control: switch view robot %s", name().c_str());
    JausAddress addr(getSubsystemID(), 255, 255);
    if (checked) {
        ui_robot_.button_view->setChecked(checked);
        view_activated(addr);
    } else {
        if (hasControl()) {
            ui_robot_.button_control->setChecked(false);
            control_deactivated(addr);
        }
        view_deactivated(addr);
    }
  }

  void AccessControlRobot::onRobotControl(bool checked)
  {
    ROS_INFO("IOP access control: switch control robot %s", name().c_str());
    JausAddress addr(getSubsystemID(), 255, 255);
    if (checked) {
        ui_robot_.button_view->setChecked(checked);
        control_activated(addr);
        // handoff_dialog_.on_access = true;
    } else {
        releaseControl();
        control_deactivated(addr);
        // handoff_dialog_.cancel_handoff();
        // handoff_dialog_.on_access = false;
    }
  }

  bool AccessControlRobot::hasControl()
  {
    return ui_robot_.button_control->isChecked();
  }

  bool AccessControlRobot::hasView()
  {
    return ui_robot_.button_view->isChecked();
  }

  void AccessControlRobot::releaseControl()
  {
    ui_robot_.button_view->setChecked(false);
    ui_robot_.button_control->setChecked(false);
  }

  void AccessControlRobot::activateView()
  {
    ui_robot_.button_view->setChecked(true);
  }

  void AccessControlRobot::onShowHandoff(bool checked)
  {

  }

  void AccessControlRobot::onShowWarnings(bool checked)
  {
    QTextBrowser* text_browser = ui_warning_info_.warnings;
    text_browser->clear();
    if (warnings_.size() == 0 && feedback_warnings_.size() == 0) {
        text_browser->append("No known warnings!");
    } else {
      for (size_t i = 0; i < warnings_.size(); i++) {
        text_browser->append(warnings_[i].c_str());
      }
      if (feedback_warnings_.size() > 0) {
        text_browser->append("Services with warning state:");
        std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator it;
        for (it = feedback_warnings_.begin(); it != feedback_warnings_.end(); ++it) {
          std::string text_client_info = "Client" + it->first.str();
          text_browser->append(text_client_info.c_str());
          for (size_t s = 0; s < it->second.size(); s++) {
            std::string text_info = it->second[s].uri;
            JausAddress ctrl_addr(it->second[s].addr_control.subsystem_id, it->second[s].addr_control.node_id, it->second[s].addr_control.component_id);
            text_info += ctrl_addr.str() + accessStateToStr(it->second[s].access_state);
            text_browser->append(text_info.c_str());
          }
        }
      }
    }
    warning_dialog_->show();
  }

  void AccessControlRobot::onShowDetails(bool checked)
  {
    ui_system_info_.treewidget_components->clear();
    std::string client_info("OCU client: ---");
    if (ocu_client_ != NULL) {
        std::string add_info;
        if (ocu_client_->restrictedToSubsystem() == getSubsystemID()) {
            if (ocu_client_->onlyMonitor()) {
                add_info = " [restricted, only monitor]";
            } else {
                add_info = " [restricted]";
            }
        }
        client_info = "OCU client: " + getControlAddr().str() + add_info;
    } else if (getSubsystemID() != 0) {
        client_info = "Controlled by other OCU: " + getControlAddr().str();
    }
    ui_system_info_.label_info->setText(client_info.c_str());
    for (size_t n = 0; n != subsystem_.nodes.size(); n++) {
        QTreeWidgetItem* node_item = new QTreeWidgetItem(ui_system_info_.treewidget_components);
        std::string node_name = subsystem_.nodes[n].ident.name;
        if (node_name.empty()) {
          node_name = "NODE";
        }
        std::string node_str = node_name + " [id: " + std::to_string(subsystem_.nodes[n].ident.address.node_id) + "]";
        node_item->setText(0, node_str.c_str());
        for (size_t c = 0; c != subsystem_.nodes[n].components.size(); c++) {
          QTreeWidgetItem* cmp_item = new QTreeWidgetItem(node_item);
          std::string cmp_name = getComponentName(subsystem_.nodes[n].components[c].address);
          JausAddress cmpaddr(subsystem_.nodes[n].components[c].address.subsystem_id, subsystem_.nodes[n].components[c].address.node_id, subsystem_.nodes[n].components[c].address.component_id);
          std::string cmp_str = cmp_name + "[" + cmpaddr.str() + "]";
          cmp_item->setText(0, cmp_str.c_str());
          ui_system_info_.treewidget_components->expandItem(node_item);
          for (size_t s = 0; s != subsystem_.nodes[n].components[c].services.size(); s++) {
            QTreeWidgetItem* srv_item = new QTreeWidgetItem(cmp_item);
            std::string srv_uri = subsystem_.nodes[n].components[c].services[s].uri;
            std::string major_version = std::to_string(subsystem_.nodes[n].components[c].services[s].major_version);
            std::string minor_version = std::to_string(subsystem_.nodes[n].components[c].services[s].minor_version);
            std::string srv_str = srv_uri + "v" + major_version + "." + minor_version;
            srv_item->setText(0, srv_str.c_str());
          }
        }
    }
    if (detailed_dialog_->isVisible()) {
        detailed_dialog_->setFocus(Qt::ActiveWindowFocusReason);
    } else {
        detailed_dialog_->show();
    }
  }

  std::string AccessControlRobot::getComponentName(fkie_iop_msgs::JausAddress address)
  {
    JausAddress cmpaddr(address.subsystem_id, address.node_id, address.component_id);
    std::map<JausAddress, std::string>::iterator itc = component_names_.find(cmpaddr);
    if (itc != component_names_.end()) {
      return component_names_[cmpaddr];
    }
    return "Component";
  }

  bool AccessControlRobot::isOld()
  {
    return ros::WallTime::now() - last_update_ > ros::WallDuration(MAX_AGE);
  }

  fkie_iop_msgs::OcuCmdEntry AccessControlRobot::stateToCmd()
  {
    fkie_iop_msgs::OcuCmdEntry cmd;
    cmd.authority = authority_;
    cmd.name = name();
    cmd.address.subsystem_id = getSubsystemID();
    cmd.address.node_id = 255;
    cmd.address.component_id = 255;
    if (ui_robot_.button_control->isChecked()) {
        cmd.access_control = 12;
    } else if (ui_robot_.button_view->isChecked()) {
        cmd.access_control = 11;
    } else {
        cmd.access_control = 10;
    }
    if (ocu_client_ != NULL) {
        //cmd.ocu_client = ocu_client.address
        cmd.ocu_client.subsystem_id = ocu_client_->getSubsystemID();
        cmd.ocu_client.node_id = 255;
        cmd.ocu_client.component_id = 255;
    } else {
        cmd.ocu_client.subsystem_id = 65535;
        cmd.ocu_client.node_id = 255;
        cmd.ocu_client.component_id = 255;
    }
    return cmd;
  }

  AccessControlClient* AccessControlRobot::ocuClientRestricted()
  {
    if (ocu_client_ != NULL) {
        if (ocu_client_->restrictedToSubsystem() == getSubsystemID()) {
            return ocu_client_;
        }
    }
    return NULL;
  }

  /**
   * @brief warnigns: dict(Address of the ocu client: OcuServiceInfo)
   * 
   */
  void AccessControlRobot::updateFeedbackWarnings()
  {
      // get all warnings for each subsystem
      int subsystem_id = getSubsystemID();
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > warnings;
      if (ocu_client_ != NULL) {
          std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > cw = ocu_client_->getWarnings(subsystem_id, hasControl());
          warnings.insert(cw.begin(), cw.end());
          // get insufficient authority reports to update handoff state button
          std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > insathority;
          std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > cw2 = ocu_client_->getSrvsInsAuthority(subsystem_id);
          insathority.insert(cw2.begin(), cw2.end());
          // update insufficient authority to activate handoff dialog
          // handoff_dialog.update_authority_problems(insathority);
      }
      feedback_warnings_ = warnings;
      updateWarningsButton();
  }

  void AccessControlRobot::setWarnings(std::vector<std::string> warnings)
  {
      warnings_.assign(warnings.begin(), warnings.end());
      updateWarningsButton();
  }

  void AccessControlRobot::updateWarningsButton()
  {
      bool has_warning = (warnings_.size() + feedback_warnings_.size()) > 0;
      if (has_warning && hasControl()) {
          ui_robot_.button_control->setStyleSheet("QPushButton { background-color: #FE9A2E;}");
      } else if (hasControl()) {
          ui_robot_.button_control->setStyleSheet("QPushButton { background-color: #98FB98;}");
          ui_robot_.button_view->setStyleSheet("QPushButton { background-color: #98FB98;}");
      } else if (hasView()) {
          ui_robot_.button_control->setStyleSheet("QPushButton { background-color: None;}");
          ui_robot_.button_view->setStyleSheet("QPushButton { background-color: #98FB98;}");
      } else if (getControlAddr().getSubsystemID() != 0 && (ocu_client_ == NULL || getControlAddr().getSubsystemID() != ocu_client_->getSubsystemID())) {
          ui_robot_.button_control->setStyleSheet("QPushButton { background-color: #A9A9A9;}");
          ui_robot_.button_view->setStyleSheet("QPushButton { background-color: None;}");
      } else {
          ui_robot_.button_control->setStyleSheet("QPushButton { background-color: None;}");
          ui_robot_.button_view->setStyleSheet("QPushButton { background-color: None;}");
      }
      ui_robot_.button_warnings->setEnabled(has_warning);
  }

  void AccessControlRobot::updateIdent(const fkie_iop_msgs::Identification::ConstPtr& msg)
  {
    JausAddress cmpaddr(msg->address.subsystem_id, msg->address.node_id, msg->address.component_id);
    JausAddress ctrladdr(getSubsystemID(), subsystem_.ident.address.node_id, subsystem_.ident.address.component_id);
    if (cmpaddr == ctrladdr) {
      last_update_ = ros::WallTime::now();
    }
    if (msg->system_type == 60001 || msg->request_type == 4) {
        if (msg->address.subsystem_id == getSubsystemID()) {
          component_names_[cmpaddr] = msg->name;
        }
    }
  }

  std::string AccessControlRobot::accessStateToStr(int state) {
    if (state == 0)
        return "NOT_AVAILABLE";
    if (state == 1)
        return "NOT_CONTROLLED";
    if (state == 2)
        return "CONTROL_RELEASED";
    if (state == 3)
        return "CONTROL_ACCEPTED";
    if (state == 4)
        return "TIMEOUT";
    if (state == 5)
        return "INSUFFICIENT_AUTHORITY";
    if (state == 6)
        return "MONITORING";
    return "UNKNOWN";
  }
}
