// *****************************************************************************
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

#ifndef FKIE_IOP_MAPVIZ_PLUGINS_ACCESS_CONTROL_CLIENT_H_
#define FKIE_IOP_MAPVIZ_PLUGINS_ACCESS_CONTROL_CLIENT_H_

// C++ standard libraries
#include <map>
#include <string>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

// ROS libraries
#include <ros/ros.h>

// IOP
#include <Transport/JausAddress.h>

// Messages
#include <fkie_iop_msgs/HandoffRequest.h>
#include <fkie_iop_msgs/HandoffResponse.h>
#include <fkie_iop_msgs/OcuFeedback.h>
#include <fkie_iop_msgs/OcuServiceInfo.h>

// QT autogenerated files
#include "ui_access_control_robot.h"
#include "ui_access_control_system_info.h"

namespace fkie_iop_mapviz_plugins
{
  class AccessControlClient : public QObject
  {
    Q_OBJECT

   public:
    int control_subsystem;  // this value is set by AccessControlRobot

    AccessControlClient(unsigned short subsystem_id, unsigned char node_id, std::string caller_ns);
    virtual ~AccessControlClient();

    JausAddress address();
    int getSubsystemID();
    int restrictedToSubsystem();
    bool onlyMonitor();
    bool hasControlAccess();
    bool hasViewAccess();
    bool isRestricted();
    bool hasHandoffPublisher();
    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo>> getWarnings(int subsystem=65535, bool has_control=false);
    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > getSrvsInsAuthority(int subsystem=65535);
    bool apply(const fkie_iop_msgs::OcuFeedback::ConstPtr& msg);

   Q_SIGNALS:
    void signal_handoffRequest(fkie_iop_msgs::HandoffRequest request);
    void signal_handoffResponse(fkie_iop_msgs::HandoffResponse respose);

   protected Q_SLOTS:
     void callbackHandoffRemoteRequest(fkie_iop_msgs::HandoffRequest msg);
     void callbackHandoffRemoteResponse(fkie_iop_msgs::HandoffResponse msg);
 
   private:
    ros::NodeHandle node_ns_;
    JausAddress address_;
    unsigned short subsystem_restricted_;
    bool only_monitor_;
    bool has_control_access_;
    bool has_view_access_;
    bool handoff_supported_;

    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > ocu_nodes_;  // address of ocu client : services
    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > warnings_;  // address of ocu client : list of services with warnings
    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > ins_autorithy_;  // address of ocu client : list of services with INSUFFICIENT_AUTHORITY

    ros::Publisher pub_handoff_own_request_;
    ros::Publisher pub_handoff_own_response_;
    ros::Subscriber sub_handoff_remote_request_;
    ros::Subscriber sub_handoff_remote_response_;
  };
}

#endif  // FKIE_IOP_MAPVIZ_PLUGINS_ACCESS_CONTROL_CLIENT_H_
