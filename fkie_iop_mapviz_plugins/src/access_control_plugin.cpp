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

#include <fkie_iop_mapviz_plugins/access_control_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

// IOP
#include <Transport/JausAddress.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fkie_iop_mapviz_plugins::AccessControlPlugin, mapviz::MapvizPlugin)


namespace fkie_iop_mapviz_plugins
{
  AccessControlPlugin::AccessControlPlugin() :
  mapviz::MapvizPlugin(),
    initialized_(false),
    timer_update_robots_(NULL),
    acdialog_(NULL),
    settings_(this->ui_),
    config_widget_(new QWidget()),
    map_canvas_(NULL)
  {
    ui_.setupUi(config_widget_);

    // ui_.color->setColor(Qt::green);
    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);
    settings_.printError("no system info received");
  }

  AccessControlPlugin::~AccessControlPlugin()
  {
    if (map_canvas_) {
      map_canvas_->removeEventFilter(this);
    }
    if (timer_update_robots_) {
      QObject::disconnect(timer_update_robots_, SIGNAL(timeout()), this, SLOT(updateRobots()));
      timer_update_robots_->stop();
    }
  }

  void AccessControlPlugin::VisibilityChanged(bool visible)
  {
    if (visible) {
      map_canvas_->installEventFilter(this);
      settings_.initTopics();
    } else {
      map_canvas_->removeEventFilter(this);
      settings_.shutdownTopics();
    }
  }

  void AccessControlPlugin::onShowAccessControl()
  {
    if (acdialog_ != NULL) {
      acdialog_->update(robotlist_);
    } else {
      acdialog_ = new AccessControlDialog(robotlist_);
      acdialog_->setAttribute(Qt::WA_DeleteOnClose);
      QObject::connect(acdialog_, SIGNAL(finished(int)), this, SLOT(onFinishedAccessControl(int)));
    }
    acdialog_->show();
  }

  void AccessControlPlugin::onFinishedAccessControl(int result)
  {
    QObject::disconnect(acdialog_, SIGNAL(finished(int)), this, SLOT(onFinishedAccessControl(int)));
    acdialog_ = NULL;
  }

  QWidget* AccessControlPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool AccessControlPlugin::Initialize(QGLWidget* canvas)
  {
    if (!initialized_) {
      map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
      map_canvas_->installEventFilter(this);
      QObject::connect(this, SIGNAL(VisibleChanged(bool)), this, SLOT(VisibilityChanged(bool)));
      QObject::connect(ui_.button_access_control, SIGNAL(clicked(bool)), this, SLOT(onShowAccessControl()));
      QObject::connect(ui_.button_release, SIGNAL(clicked(bool)), this, SLOT(releaseAll()));
      QObject::connect(&settings_, SIGNAL(signal_system(IopSystemPtr)), this, SLOT(onIopSystem(IopSystemPtr)));
      QObject::connect(&settings_, SIGNAL(signal_feedback(IopOcuFeedbackPtr, std::string)), this, SLOT(onIopFeedback(IopOcuFeedbackPtr, std::string)));
      QObject::connect(&settings_, SIGNAL(signal_ident(IopIdentificationPtr)), this, SLOT(onIopIdent(IopIdentificationPtr)));
      QObject::connect(&settings_, SIGNAL(signal_control_report(IopControlReportPtr, std::string)), this, SLOT(onIopControlReport(IopControlReportPtr, std::string)));

      timer_update_robots_ = new QTimer(this);
      QObject::connect(timer_update_robots_, SIGNAL(timeout()), this, SLOT(updateRobots()));
      settings_.initialize();
      initialized_ = true;
      timer_update_robots_->start(1000);
    }
    return true;
  }



  void AccessControlPlugin::onIopSystem(boost::shared_ptr<const fkie_iop_msgs::System_<std::allocator<void> > > msg)
  {
    settings_.printInfo("system update received");
//    fkie_iop_msgs::System* msg = system.value<fkie_iop_msgs::System*>();
    for (size_t i = 0; i != msg->subsystems.size(); i++) {
      bool updated = false;
      std::vector<AccessControlRobot *>::iterator it;
      for (it = robotlist_.begin(); it != robotlist_.end(); ++it) {
        if (!updated && (*it)->updateSubsystem(msg->subsystems[i])) {
              updated = true;
        }
      }
      if (!updated) {
        AccessControlRobot* robot = new AccessControlRobot(msg->subsystems[i], ui_.authority_edit->text().toInt());
        QObject::connect(robot, SIGNAL(control_activated(JausAddress)), this, SLOT(onRobotControlActivated(JausAddress)));
        QObject::connect(robot, SIGNAL(control_deactivated(JausAddress)), this, SLOT(onRobotControlDeactivated(JausAddress)));
        QObject::connect(robot, SIGNAL(view_activated(JausAddress)), this, SLOT(onRobotViewActivated(JausAddress)));
        QObject::connect(robot, SIGNAL(view_deactivated(JausAddress)), this, SLOT(onRobotViewDeactivated(JausAddress)));
        robotlist_.push_back(robot);
        ui_.robot_stats->setText(std::to_string(robotlist_.size()).c_str());
        // set an already discovered client if it is restricted to this robot
        std::vector<AccessControlClient *>::iterator itc;
        for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
            int ssms = (*itc)->restrictedToSubsystem();
            if (ssms == robot->getSubsystemID()) {
              if (robot->getOcuClient() == NULL) {
                robot->setOcuClient(*itc);
              }
            }
        }
      }
    }
    if (msg->subsystems.size() == 0) {
      settings_.printWarning("no IOP compliant robots available");
    }
  }

  /**
   * @brief apply feedback to the clients
   * 
   * @param msg: ROS message
   * @param caller_ns: the namespce of the IOP client node
   */
  void AccessControlPlugin::onIopFeedback(IopOcuFeedbackPtr msg, std::string caller_ns)
  {
    // find the existing client or create a new one
    AccessControlClient* client = NULL;
    JausAddress caddr(msg->reporter.subsystem_id, msg->reporter.node_id, 255);
    client = getClient(caddr);
    if (client == NULL) {
      client = new AccessControlClient(caddr.getSubsystemID(), caddr.getNodeID(), caller_ns);
      QObject::connect(client, SIGNAL(signal_handoffRequest(fkie_iop_msgs::HandoffRequest)), this, SLOT(handoffRequest(fkie_iop_msgs::HandoffRequest)));
      QObject::connect(client, SIGNAL(signal_handoffResponse(fkie_iop_msgs::HandoffResponse)), this, SLOT(handoffResponse(fkie_iop_msgs::HandoffResponse)));
      clientlist_.push_back(client);
      //self._clients.sort()
    }
    client->apply(msg);
    ui_.ocu_clients->setText(std::to_string(clientlist_.size()).c_str());
    // handle feedback of OCU clients
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
      (*itr)->updateFeedbackWarnings();
    }
  }

  AccessControlClient* AccessControlPlugin::getClient(JausAddress addr)
  { 
    std::vector<AccessControlClient *>::iterator itc;
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
      if ((*itc)->address() == addr) {
        return *itc;
      }
    }
    return NULL;
  }

  void AccessControlPlugin::onIopIdent(IopIdentificationPtr ident)
  {
    // update robot alive
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
      (*itr)->updateIdent(ident);
    }
  }

  /**
   * @brief apply feedback to the clients
   * @param controller: ROS OcuControlReport message
   */
  void AccessControlPlugin::onIopControlReport(IopControlReportPtr msg)
  {
    // find the existing client or create a new one
    JausAddress cmpaddr(msg->component.subsystem_id, msg->component.node_id, msg->component.component_id);
    JausAddress ctrladdr(msg->controller.subsystem_id, msg->controller.node_id, msg->controller.component_id);
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
      if ((*itr)->getSubsystemID() == cmpaddr.getSubsystemID()) {
        (*itr)->setControlAddr(ctrladdr);
      }
    }
  }

  void AccessControlPlugin::handoffRequest(fkie_iop_msgs::HandoffRequest request)
  {
    // for robot in self._robotlist:
    //     if robot.subsystem_id == request.component.subsystem_id:
    //         robot.handoff_dialog.handle_handoffRequest(request)
  }

  void AccessControlPlugin::handoffResponse(fkie_iop_msgs::HandoffResponse response)
  {
      // for robot in self._robotlist:
      //     if robot.subsystem_id == response.component.subsystem_id:
      //         robot.handoff_dialog.handle_handoffResponse(response)
  }

  /**
   * @param address: Normally only the subsystem ID is set.
   **/
  void AccessControlPlugin::onRobotControlActivated(JausAddress address)
  {
    AccessControlClient* control_ocu = getClientForControl(address.getSubsystemID());
    //control_client_ = control_ocu;
    std::vector<int> deactivated_robot_id;
    // release current control or view
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
        if (control_ocu == (*itr)->getOcuClient()) {
            if ((*itr)->getSubsystemID() != address.getSubsystemID()) {
                (*itr)->releaseControl();
                fkie_iop_msgs::OcuCmd cmd_release;
                fkie_iop_msgs::OcuCmdEntry cmd_entry1 = (*itr)->stateToCmd();
                cmd_entry1.access_control = ACCESS_CONTROL_RELEASE;
                if ((*itr)->getOcuClient() != NULL) {
                    if (!(*itr)->getOcuClient()->isRestricted()) {
                        (*itr)->setOcuClient(NULL);
                    }
                }
                cmd_release.cmds.push_back(cmd_entry1);
                settings_.publishCmd(cmd_release);
                deactivated_robot_id.push_back((*itr)->getSubsystemID());
            }
        }
        if ((*itr)->getSubsystemID() == address.getSubsystemID()) {
            if ((*itr)->getOcuClient() != NULL) {
                fkie_iop_msgs::OcuCmd cmd_release;
                fkie_iop_msgs::OcuCmdEntry cmd_entry1 = (*itr)->stateToCmd();
                cmd_entry1.access_control = ACCESS_CONTROL_RELEASE;
                if ((*itr)->getOcuClient() != NULL) {
                    if (!(*itr)->getOcuClient()->isRestricted()) {
                        (*itr)->setOcuClient(NULL);
                    }
                }
                cmd_release.cmds.push_back(cmd_entry1);
                settings_.publishCmd(cmd_release);
            }
        }
    }
    // set the ocu client for the new robot
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
        if ((*itr)->getSubsystemID() == address.getSubsystemID()) {
            (*itr)->setOcuClient(control_ocu);
            std::string ocu_str_addr = "---";
            if ((*itr)->getOcuClient() != NULL) {
                if (!(*itr)->getOcuClient()->isRestricted()) {
                    ocu_str_addr = (*itr)->getOcuClient()->address().str();
                }
            }
            //ui_.label_address->setText(ocu_str_addr);
        }
    }
    // create command for new robot and try to find the view ocu client for deactivated control
    fkie_iop_msgs::OcuCmd cmd;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
        // change deactivated robot to view mode?
        // if (*itr)->subsystem_id in deactivated_robot_id:
        //     (*itr)->setOcuClient(self.getClientForView(deactivated_robot_id))
        //     if (*itr)->getOcuClient() is not None:
        //         (*itr)->activateView()
        fkie_iop_msgs::OcuCmdEntry cmd_entry = (*itr)->stateToCmd();
        cmd.cmds.push_back(cmd_entry);
    }
    settings_.publishCmd(cmd);
  }

  /**
   * :param address: address of the robot. Normally only the subsystem ID is set
   * :type address: Address
   **/
  void AccessControlPlugin::onRobotControlDeactivated(JausAddress address)
  {
    fkie_iop_msgs::OcuCmd cmd;
    std::vector<AccessControlRobot *>::iterator it;
    for (it = robotlist_.begin(); it != robotlist_.end(); ++it) {
        fkie_iop_msgs::OcuCmdEntry cmd_entry = (*it)->stateToCmd();
        if ((*it)->getSubsystemID() == address.getSubsystemID()) {
            (*it)->setOcuClient(NULL);
            cmd_entry.access_control = ACCESS_CONTROL_RELEASE;
//            ui_.label_address->setText("---")
        }
        cmd.cmds.push_back(cmd_entry);
    }
    settings_.publishCmd(cmd);

  }

  void AccessControlPlugin::onRobotViewActivated(JausAddress address)
  {
    AccessControlClient* ocu_view_client = getClientForView(address.getSubsystemID());
    if (ocu_view_client != NULL) {
      fkie_iop_msgs::OcuCmd cmd;
      std::vector<AccessControlRobot *>::iterator it;
      for (it = robotlist_.begin(); it != robotlist_.end(); ++it) {
        // TODO determine the client for view
        if ((*it)->getSubsystemID() == address.getSubsystemID()) {
          (*it)->setOcuClient(ocu_view_client);
          fkie_iop_msgs::OcuCmdEntry cmd_entry = (*it)->stateToCmd();
          cmd.cmds.push_back(cmd_entry);
        }
        settings_.publishCmd(cmd);
      }
    }
  }

  void AccessControlPlugin::onRobotViewDeactivated(JausAddress address)
  {
    fkie_iop_msgs::OcuCmd cmd;
    std::vector<AccessControlRobot *>::iterator it;
    for (it = robotlist_.begin(); it != robotlist_.end(); ++it) {
      // TODO determine the client for view
      if ((*it)->getSubsystemID() == address.getSubsystemID()) {
        (*it)->setOcuClient(NULL);
        fkie_iop_msgs::OcuCmdEntry cmd_entry = (*it)->stateToCmd();
        cmd.cmds.push_back(cmd_entry);
      }
      settings_.publishCmd(cmd);
    }
  }

  void AccessControlPlugin::updateRobots()
  {
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
      if ((*itr)->isOld()) {
        // TODO: should we release control?
        // (*itr)->releaseControl();
        // fkie_iop_msgs::OcuCmd cmd_release;
        // fkie_iop_msgs::OcuCmdEntry cmd_entry1 = (*itr)->stateToCmd();
        // cmd_entry1.access_control = ACCESS_CONTROL_RELEASE;
        // if ((*itr)->getOcuClient() != NULL && !(*itr)->getOcuClient()->isRestricted()) {
        //     (*itr)->setOcuClient(NULL);
        // }
        // cmd_release.cmds.push_back(cmd_entry1);
        // settings_.publishCmd(cmd_release);
        if ((*itr)->parent() != NULL) {
          std::vector<std::string> warnings;
          warnings.push_back("timeout for identification messages");
          (*itr)->setWarnings(warnings);
        }
      }
    }
  }

  AccessControlClient* AccessControlPlugin::getClientForControl(int subsystem)
  {
    // do we have already a client assigned
    std::vector<AccessControlRobot *>::iterator itr;
    for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
        if ((*itr)->getSubsystemID() == subsystem) {
            AccessControlClient* result = (*itr)->ocuClientRestricted();
            if (result != NULL) {
                return result;
            }
        }
    }
    // look for control client with restriction to the given subsystem
    std::vector<AccessControlClient *>::iterator itc;
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
        int ssms = (*itc)->restrictedToSubsystem();
        if (ssms == subsystem && !(*itc)->onlyMonitor()) {
            return *itc;
        }
    }
    // take first available
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
        int ssms = (*itc)->restrictedToSubsystem();
        if (ssms == 65535 && !(*itc)->onlyMonitor()) {
            return *itc;
        }
    }
    settings_.printError("No free OCU client found");
    return NULL;
  }

  AccessControlClient* AccessControlPlugin::getClientForView(int subsystem)
  {
    // already assigned?
    std::vector<AccessControlClient *>::iterator itc;
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
        if ((*itc)->control_subsystem == subsystem) {
            return *itc;
        }
    }
    // look for control client with restricted to the given subsystem
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
        int ssms = (*itc)->restrictedToSubsystem();
        if (ssms == subsystem && (*itc)->control_subsystem == -1) {
            return *itc;
        }
    }
    // take first available
    for (itc = clientlist_.begin(); itc != clientlist_.end(); ++itc) {
        int ssms = (*itc)->restrictedToSubsystem();
        if (ssms == 65535 && (*itc)->control_subsystem == -1) {
            return *itc;
        }
    }
    settings_.printError("No free OCU client found");
    return NULL;
  }

  void AccessControlPlugin::releaseAll()
  {
      fkie_iop_msgs::OcuCmd cmd;
      std::vector<AccessControlRobot *>::iterator itr;
      for (itr = robotlist_.begin(); itr != robotlist_.end(); ++itr) {
          fkie_iop_msgs::OcuCmdEntry cmd_entry = (*itr)->stateToCmd();
          (*itr)->releaseControl();
          (*itr)->setOcuClient(NULL);
          cmd_entry.access_control = ACCESS_CONTROL_RELEASE;
          //ui_.label_address->setText("---");
          cmd.cmds.push_back(cmd_entry);
      }
      settings_.publishCmd(cmd);
  }

  void AccessControlPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    settings_.loadConfig(node, path);
  }

  void AccessControlPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    settings_.saveConfig(emitter, path);
  }
}
