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

#include <fkie_iop_mapviz_plugins/access_control_settings.h>


namespace fkie_iop_mapviz_plugins
{
  AccessControlSettings::AccessControlSettings(Ui::AccessControl& ui) :
    QObject(),
    initialized_(false),
    ui_(ui),
    node_ns_(NULL),
    authority_(205),
    namespace_("/"),
    service_update_("iop_update_discovery"),
    topic_discovery_("iop_system"),
    topic_identification_("iop_identification"),
    topic_command_("ocu_cmd"),
    topic_feedback_("ocu_feedback"),
    topic_control_report_("ocu_control_report")
  {
  }

  AccessControlSettings::~AccessControlSettings()
  {
    ROS_INFO("IOP access control: delete access control settings");
    delete node_ns_;
  }

  void AccessControlSettings::initialize()
  {
    if (!initialized_) {
      initTopics();
      initialized_ = true;
    }
  }

  void AccessControlSettings::initTopics()
  {
      if (namespace_ != ui_.namespace_edit->text().toStdString())
      {
        namespace_ = ui_.namespace_edit->text().toStdString();
        shutdownTopics();
        delete node_ns_;
        node_ns_ = NULL;
      }
      if (node_ns_ == NULL) {
        node_ns_ = new ros::NodeHandle(namespace_.c_str());
        pub_cmd_ = node_ns_->advertise<fkie_iop_msgs::OcuCmd>(topic_command_, 1, true);
        ROS_INFO("IOP access control: advertising %s", pub_cmd_.getTopic().c_str());
        sub_discovery_ = node_ns_->subscribe<fkie_iop_msgs::System>(topic_discovery_, 100, &AccessControlSettings::callbackIopSystem, this);
        ROS_INFO("IOP access control: subscribing to %s", sub_discovery_.getTopic().c_str());
        sub_feedback_ = node_ns_->subscribe(topic_feedback_, 5, &AccessControlSettings::callbackIopFeedback, this);
        ROS_INFO("IOP access control: subscribing to %s", sub_feedback_.getTopic().c_str());
        sub_identification_ = node_ns_->subscribe<fkie_iop_msgs::Identification>(topic_identification_, 100, &AccessControlSettings::callbackIopIdent, this);
        ROS_INFO("IOP access control: subscribing to %s", sub_identification_.getTopic().c_str());
        sub_control_report_ = node_ns_->subscribe<fkie_iop_msgs::OcuControlReport>(topic_control_report_, 100, &AccessControlSettings::callbackIopControlReport, this);
        ROS_INFO("IOP access control: subscribing to %s", sub_control_report_.getTopic().c_str());
      }
  }

  void AccessControlSettings::shutdownTopics()
  {
      pub_cmd_.shutdown();
      sub_discovery_.shutdown();
      sub_identification_.shutdown();
      sub_feedback_.shutdown();
      sub_control_report_.shutdown();
  }

  void AccessControlSettings::callbackIopSystem(const fkie_iop_msgs::System::ConstPtr& msg)
  {
    signal_system(msg);
  }

  void AccessControlSettings::callbackIopFeedback(const ros::MessageEvent<const fkie_iop_msgs::OcuFeedback>& event)
  {
    const std::string& caller_ns = event.getPublisherName();
    //const ros::M_string& header = event.getConnectionHeader();
    const fkie_iop_msgs::OcuFeedback::ConstPtr& msg = event.getMessage();    
    signal_feedback(msg, caller_ns);
  }

  void AccessControlSettings::callbackIopIdent(const fkie_iop_msgs::Identification::ConstPtr& msg)
  {
    signal_ident(msg);
  }

  void AccessControlSettings::callbackIopControlReport(const fkie_iop_msgs::OcuControlReport::ConstPtr& msg)
  {
    signal_control_report(msg);
  }

  void AccessControlSettings::printError(const std::string& message)
  {
    float throttle = 1.0;
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    if( throttle > 0.0){
        ROS_ERROR_THROTTLE(throttle, "Error: %s", message.c_str());
    }
    else{
        ROS_ERROR("Error: %s", message.c_str());
    }
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void AccessControlSettings::printInfo(const std::string& message)
  {
    float throttle = 1.0;
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    if( throttle > 0.0){
        ROS_INFO_THROTTLE(throttle, "%s", message.c_str());
    }
    else{
        ROS_INFO("%s", message.c_str());
    }
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkGreen);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void AccessControlSettings::printWarning(const std::string& message)
  {
    float throttle = 1.0;
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    if( throttle > 0.0){
        ROS_WARN_THROTTLE(throttle, "%s", message.c_str());
    }
    else{
        ROS_WARN("%s", message.c_str());
    }
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void AccessControlSettings::loadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["authority"])
    {
      std::string authority;
      node["authority"] >> authority;
      ui_.authority_edit->setText(authority.c_str());
    }
    if (node["topic_namespace"])
    {
      std::string topic_namespace;
      node["topic_namespace"] >> topic_namespace;
      ui_.namespace_edit->setText(topic_namespace.c_str());
    }
    if (node["autorequest"])
    {
      bool autorequest;
      node["autorequest"] >> autorequest;
      ui_.autorequest_cb->setChecked(autorequest);
    }
    initTopics();
  }

  void AccessControlSettings::saveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string authority = ui_.authority_edit->text().toStdString();
    emitter << YAML::Key << "autorithy" << YAML::Value << authority;

    std::string topic_namespace = ui_.namespace_edit->text().toStdString();
    emitter << YAML::Key << "topic_namespace" << YAML::Value << topic_namespace;

    bool autorequest = ui_.autorequest_cb->isChecked();
    emitter << YAML::Key << "autorequest" << YAML::Value << autorequest;
    initTopics();
  }

  void AccessControlSettings::publishCmd(fkie_iop_msgs::OcuCmd& cmd)
  {
    pub_cmd_.publish(cmd);
  }
}
