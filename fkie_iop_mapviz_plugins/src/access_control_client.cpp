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

#include <fkie_iop_mapviz_plugins/access_control_client.h>

// C++ standard libraries
#include <cstdio>
#include <vector>


namespace fkie_iop_mapviz_plugins
{
  AccessControlClient::AccessControlClient(unsigned short subsystem_id, unsigned char node_id, std::string caller_ns) :
    QObject(),
    control_subsystem(-1),
    node_ns_(caller_ns),
    address_(subsystem_id, node_id, 255),
    subsystem_restricted_(65535),
    only_monitor_(false),
    has_control_access_(false),
    has_view_access_(false),
    handoff_supported_(false)
  {
    std::string topic_handoff_own_request = caller_ns + "/handoff_own_request";
    std::string topic_handoff_own_response = caller_ns + "/handoff_own_response";
    std::string topic_handoff_remote_request = caller_ns + "/handoff_remote_request";
    std::string topic_handoff_remote_response = caller_ns + "/handoff_remote_response";

    pub_handoff_own_request_ = node_ns_.advertise<fkie_iop_msgs::HandoffRequest>(topic_handoff_own_request, 1, true);
    pub_handoff_own_response_ = node_ns_.advertise<fkie_iop_msgs::HandoffResponse>(topic_handoff_own_response, 1, true);
    sub_handoff_remote_request_ = node_ns_.subscribe<fkie_iop_msgs::HandoffRequest>(topic_handoff_remote_request, 100, &AccessControlClient::callbackHandoffRemoteRequest, this);
    sub_handoff_remote_response_ = node_ns_.subscribe<fkie_iop_msgs::HandoffResponse>(topic_handoff_remote_response, 100, &AccessControlClient::callbackHandoffRemoteResponse, this);
  }

  AccessControlClient::~AccessControlClient()
  {
    ROS_INFO("IOP access control: delete access control client %d.%d", address_.getSubsystemID(), address_.getNodeID());
    pub_handoff_own_request_.shutdown();
    pub_handoff_own_response_.shutdown();
    sub_handoff_remote_request_.shutdown();
    sub_handoff_remote_response_.shutdown();
    ocu_nodes_.clear();
    warnings_.clear();
    ins_autorithy_.clear();
  }

  JausAddress AccessControlClient::address()
  {
    return address_;
  }

  int AccessControlClient::getSubsystemID()
  {
    return address_.getSubsystemID();
  }

  int AccessControlClient::restrictedToSubsystem()
  {
    return subsystem_restricted_;
  }

  bool AccessControlClient::onlyMonitor()
  {
    return only_monitor_;
  }

  bool AccessControlClient::hasControlAccess()
  {
    return has_control_access_;
  }

  bool AccessControlClient::hasViewAccess()
  {
    return has_view_access_;
  }

  bool AccessControlClient::isRestricted()
  {
    return (subsystem_restricted_ != 0 && subsystem_restricted_ != 65535);
  }

  bool AccessControlClient::hasHandoffPublisher()
  {
    return (sub_handoff_remote_request_.getNumPublishers() > 0 && pub_handoff_own_request_.getNumSubscribers() > 0);
  }

  void AccessControlClient::callbackHandoffRemoteRequest(fkie_iop_msgs::HandoffRequest msg)
  {
    signal_handoffRequest(msg);
  }

  void AccessControlClient::callbackHandoffRemoteResponse(fkie_iop_msgs::HandoffResponse msg)
  {
    signal_handoffResponse(msg);
  }

  /**
   * @brief Returns warnings for controlled subsystem. Returns all warnings if no subsystem specified.
   * 
   * @param subsystem: 65535 for all warnings
   * @param has_control: 
   * @return std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > 
   */
  std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > AccessControlClient::getWarnings(int subsystem, bool has_control)
  {
      if (subsystem == 65535) {
          return warnings_;
      }
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > result;
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itw;
      for (itw = warnings_.begin(); itw != warnings_.end(); ++itw) {
          JausAddress address = itw->first;
          std::vector<fkie_iop_msgs::OcuServiceInfo>::iterator its;
          for (its = itw->second.begin(); its != itw->second.end(); ++its) {
              if ((*its).addr_control.subsystem_id == subsystem) {
                  std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itf = result.find(address);
                  if (itf == result.end()) {
                    result[address] = std::vector<fkie_iop_msgs::OcuServiceInfo>();
                  }
                  result[address].push_back(*its);
              }
          }
      }
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itn;
      for (itn = ocu_nodes_.begin(); itn != ocu_nodes_.end(); ++itn) {
          JausAddress address = itn->first;
          std::vector<fkie_iop_msgs::OcuServiceInfo>::iterator its;
          for (its = itn->second.begin(); its != itn->second.end(); ++its) {
              if ((*its).addr_control.subsystem_id == subsystem) {
                  if (has_control && (*its).access_state == 2) {  // see OcuServiceInfo for number
                      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itf = result.find(address);
                      if (itf == result.end()) {
                          result[address] = std::vector<fkie_iop_msgs::OcuServiceInfo>();
                      }
                      result[address].push_back(*its);
                  }
              }
          }
      }
      return result;
  }

  /**
   * @brief Returns warnings for controlled subsystem. Returns all warnings if no subsystem specified.
   * 
   * @param subsystem: 65535 for all
   * @return std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > 
   */
  std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > AccessControlClient::getSrvsInsAuthority(int subsystem)
  {
      if (subsystem == 65535) {
          return ins_autorithy_;
      }
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> > result;
      std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itw;
      for (itw = ins_autorithy_.begin(); itw != ins_autorithy_.end(); ++itw) {
          JausAddress address = itw->first;
          std::vector<fkie_iop_msgs::OcuServiceInfo>::iterator its;
          for (its = itw->second.begin(); its != itw->second.end(); ++its) {
              if ((*its).addr_control.subsystem_id == subsystem) {
                  std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itf = result.find(address);
                  if (itf == result.end()) {
                    result[address] = std::vector<fkie_iop_msgs::OcuServiceInfo>();
                  }
                  result[address].push_back(*its);
              }
          }
      }
      return result;
  }

  bool AccessControlClient::apply(const fkie_iop_msgs::OcuFeedback::ConstPtr& msg)
  {
    JausAddress jaus_address(msg->reporter.subsystem_id, msg->reporter.node_id, 255);
    if (address_ != jaus_address) {
        return false;
    }
    // change the controlled subsystem only once to avoid glint
    if (subsystem_restricted_ == 65535) {
        if (msg->subsystem_restricted == 0 || msg->subsystem_restricted == 65535) {
            subsystem_restricted_ = msg->subsystem_restricted;
        }
    }
    if (!only_monitor_ && msg->only_monitor) {
        only_monitor_ = msg->only_monitor;
    }
    ocu_nodes_[jaus_address] = msg->services;
    warnings_.clear();
    ins_autorithy_.clear();
    has_control_access_ = false;
    has_view_access_ = false;
    handoff_supported_ = msg->handoff_supported;
    std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itn;
    for (itn = ocu_nodes_.begin(); itn != ocu_nodes_.end(); ++itn) {
      std::vector<fkie_iop_msgs::OcuServiceInfo>::iterator its;
      for (its = itn->second.begin(); its != itn->second.end(); ++its) {
        // address = Address(msg->reporter)
        int state = (*its).access_state;
        if (state == 0 || state == 4 || state == 5) {  // see OcuServiceInfo for number
          std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itf = warnings_.find(jaus_address);
          if (itf == warnings_.end()) {
            warnings_[jaus_address] = std::vector<fkie_iop_msgs::OcuServiceInfo>();
          }
          warnings_[jaus_address].push_back(*its);
//                if service_info.access_state in [3, 6]:  # see OcuServiceInfo for number
//                    self._assined_subsystems.add(service_info.addr_control.subsystem_id)
        }
        if (state == 5) { // ACCESS_STATE_INSUFFICIENT_AUTHORITY
          std::map<JausAddress, std::vector<fkie_iop_msgs::OcuServiceInfo> >::iterator itf = ins_autorithy_.find(jaus_address);
          if (itf == ins_autorithy_.end()) {
            ins_autorithy_[jaus_address] = std::vector<fkie_iop_msgs::OcuServiceInfo>();
          }
          ins_autorithy_[jaus_address].push_back(*its);
        }
        if (state == 3) {  // see OcuServiceInfo for number
          has_control_access_ = true;
          control_subsystem = (*its).addr_control.subsystem_id;
        }
        if (state == 6) {  // see OcuServiceInfo for number
          has_view_access_ = true;
          control_subsystem = (*its).addr_control.subsystem_id;
        }
      }
    }
    return true;
  }

}
