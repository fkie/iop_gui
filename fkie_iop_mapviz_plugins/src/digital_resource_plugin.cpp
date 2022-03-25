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

#include <fkie_iop_mapviz_plugins/digital_resource_plugin.h>

// C++ standard libraries
#include <algorithm>
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

#include <mapviz/select_topic_dialog.h>
#include <mapviz/select_frame_dialog.h>

// IOP
#include <Transport/JausAddress.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fkie_iop_mapviz_plugins::DigitalResourcePlugin, mapviz::MapvizPlugin)


namespace fkie_iop_mapviz_plugins
{
  struct less_than_key
  {
      inline bool operator() (DigitalResourceCam*& cam1, DigitalResourceCam*& cam2)
      {
          return (cam1->text() < cam2->text());
      }
  };

  DigitalResourcePlugin::DigitalResourcePlugin() :
  mapviz::MapvizPlugin(),
    initialized_(false),
    config_widget_(new QWidget()),
    map_canvas_(NULL),
    info_dialog_(NULL),
    node_ns_(NULL),
    multiurl_(false),
    topic_video_url_("current_video_url"),
    topic_endpoints_("digital_endpoints"),
    topic_names_("visual_sensor_names"),
    topic_resource_id("dv_resource_id"),
    ns_("/")
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
    topics_.push_back("fkie_iop_msgs/DigitalResourceEndpoints");
    topics_.push_back("fkie_iop_msgs/VisualSensorNames");
    QObject::connect(this, &DigitalResourcePlugin::VisibleChanged, this, &DigitalResourcePlugin::setVisible);
    QObject::connect(ui_.selectns, &QPushButton::clicked, this, &DigitalResourcePlugin::selectNS);
    QObject::connect(ui_.ns, &QLineEdit::editingFinished, this, &DigitalResourcePlugin::nsEdited);
    QObject::connect(this, &DigitalResourcePlugin::signalIopEndpoints, this, &DigitalResourcePlugin::onIopEndpoints);
    QObject::connect(this, &DigitalResourcePlugin::signalIopVisualNames, this, &DigitalResourcePlugin::onIopVisualNames);
    QObject::connect(ui_.buttonInfo, &QPushButton::clicked, this, &DigitalResourcePlugin::showInfo);
  }

  DigitalResourcePlugin::~DigitalResourcePlugin()
  {
    if (map_canvas_) {
      map_canvas_->removeEventFilter(this);
    }
    shutdownTopics();
    shutdownRidTopics();
    // if (timer_update_robots_) {
    //   QObject::disconnect(timer_update_robots_, SIGNAL(timeout()), this, SLOT(updateRobots()));
    //   timer_update_robots_->stop();
    // }
  }

  void DigitalResourcePlugin::selectNS()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(topics_);
    if (topic.name.empty())
    {
      return;
    }

    std::vector<std::string> tokens = split(topic.name, '/');
    std::string ns;
    for (size_t i = 0; i < tokens.size()-1; i++) {
      ROS_INFO("JOIN %s", tokens[i].c_str());
      ns += tokens[i] + "/";
    }
    ui_.ns->setText(QString::fromStdString(ns));
    nsEdited();
  }

  void DigitalResourcePlugin::nsEdited()
  {
    if (!ui_.ns->text().endsWith('/')) {
      ui_.ns->setText(ui_.ns->text().append('/'));
    }
    std::string topic = ui_.ns->text().trimmed().toStdString();
    if (topic != ns_)
    {
      PrintWarning("No messages received.");
      initTopics();
      ns_ = topic;
    }
  }

  void DigitalResourcePlugin::stateChangedMU(int state)
  {
    if (state == Qt::Unchecked) {
      multiurl_ = false;
    } else if (state == Qt::Checked) {
      multiurl_ = true;
    }
  }

  void DigitalResourcePlugin::setVisible(bool visible)
  {
    if (visible) {
      map_canvas_->installEventFilter(this);
      initTopics();
    } else {
      map_canvas_->removeEventFilter(this);
      shutdownTopics();
    }
  }

  QWidget* DigitalResourcePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool DigitalResourcePlugin::Initialize(QGLWidget* canvas)
  {
    if (!initialized_) {
      map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
      map_canvas_->installEventFilter(this);
      // QObject::connect(ui_.buttonInfo, SIGNAL(clicked(bool)), this, SLOT(onShowInfo()));

      // timer_update_robots_ = new QTimer(this);
      // QObject::connect(timer_update_robots_, SIGNAL(timeout()), this, SLOT(updateRobots()));
      // settings_.initialize();
      // timer_update_robots_->start(1000);
      initTopics();
      initialized_ = true;
    }
    return true;
  }

  void DigitalResourcePlugin::initTopics()
  {
      if (ns_ != ui_.ns->text().toStdString())
      {
        ns_ = ui_.ns->text().toStdString();
        shutdownTopics();
      }
      if (node_ns_ == NULL) {
        PrintError("no system info received");
        node_ns_ = new ros::NodeHandle(ns_.c_str());
        pub_resource_id_ = node_ns_->advertise<std_msgs::UInt16>(topic_resource_id, 1, true);
        ROS_INFO("IOP digital resource: advertising %s", pub_resource_id_.getTopic().c_str());
        pub_video_url_ = node_ns_->advertise<std_msgs::String>(topic_video_url_, 1, true);
        ROS_INFO("IOP digital resource: advertising %s", pub_video_url_.getTopic().c_str());
        sub_endpoints_ = node_ns_->subscribe<fkie_iop_msgs::DigitalResourceEndpoints>(topic_endpoints_, 100, &DigitalResourcePlugin::callbackIopEndpoints, this);
        ROS_INFO("IOP digital resource: subscribing to %s", sub_endpoints_.getTopic().c_str());
        sub_names_ = node_ns_->subscribe<fkie_iop_msgs::VisualSensorNames>(topic_names_, 100, &DigitalResourcePlugin::callbackIopVisualNames, this);
        ROS_INFO("IOP digital resource: subscribing to %s", sub_names_.getTopic().c_str());
        // TODO: handle multiurl: init Publisher for each resource id
      }
  }

  void DigitalResourcePlugin::shutdownTopics()
  {
      pub_resource_id_.shutdown();
      pub_video_url_.shutdown();
      sub_endpoints_.shutdown();
      sub_names_.shutdown();
      if (node_ns_ != NULL) {
        delete node_ns_;
        node_ns_ = NULL;
      }
  }

  void DigitalResourcePlugin::shutdownRidTopics()
  {
    std::map<std::string, ros::Publisher>::iterator itp;
    for (itp = pub_video_urls_.begin(); itp != pub_video_urls_.end(); ++itp) {
      itp->second.shutdown();
    }
    pub_video_urls_.clear();
  }

  void DigitalResourcePlugin::callbackIopEndpoints(const fkie_iop_msgs::DigitalResourceEndpoints::ConstPtr& msg)
  {
    signalIopEndpoints(msg);
  }

  void DigitalResourcePlugin::callbackIopVisualNames(const fkie_iop_msgs::VisualSensorNames::ConstPtr& msg)
  {
    signalIopVisualNames(msg);
  }

  void DigitalResourcePlugin::onIopEndpoints(IopEndpointsPtr msg)
  {
    PrintInfo(std::to_string(msg->endpoints.size()) + " endpoints discovered");
    // update endpoints
    std::vector<DigitalResourceCam*> new_cam_list;
    std::vector<DigitalResourceCam*>::iterator itc;
    for (itc = cam_list_.begin(); itc < cam_list_.end(); ++itc) {
      try {
        ui_.layoutButtons->removeWidget(*itc);
        if (!isIn((*itc)->getResourceId(), msg)) {
          (*itc)->setParent(NULL);
          // remove publisher if exists for this cam id
          std::map<std::string, ros::Publisher>::iterator itp = pub_video_urls_.find((*itc)->getServerUrl());
          if (itp != pub_video_urls_.end()) {
            std_msgs::String ros_msg_str;
            itp->second.publish(ros_msg_str);
            itp->second.shutdown();
            pub_video_urls_.erase(itp);
          }
        } else {
          new_cam_list.push_back((*itc));
        }
      } catch (const std::exception& e) {
        ROS_WARN("Error: %s", e.what());
      }
    }
    for (size_t i = 0; i < msg->endpoints.size(); i++) {
      DigitalResourceCam* new_cam = new DigitalResourceCam(msg->endpoints[i], getResourceName(msg->endpoints[i].resource_id));
      if (!isIn(new_cam->getResourceId(), new_cam_list)) {
        QObject::connect(new_cam, &DigitalResourceCam::signalPlay, this, &DigitalResourcePlugin::play);
        QObject::connect(new_cam, &DigitalResourceCam::signalStop, this, &DigitalResourcePlugin::stop);
        new_cam_list.push_back(new_cam);
        std::map<std::string, ros::Publisher>::iterator it = pub_video_urls_.find(new_cam->getServerUrl());
        if (it != pub_video_urls_.end()) {
          new_cam->setPlayed(true);
        }
      }
    }
    // (re)add all items in sorted order
    std::sort(new_cam_list.begin(), new_cam_list.end(), less_than_key());
    cam_list_.assign(new_cam_list.begin(), new_cam_list.end());
    std::vector<DigitalResourceCam*>::iterator itn;
    for (itn = cam_list_.begin(); itn < cam_list_.end(); ++itn) {
      ui_.layoutButtons->addWidget(*itn);
      // create publisher if not exists
      std::string server_url = (*itn)->getServerUrl();
      std::map<std::string, ros::Publisher>::iterator it = pub_video_urls_.find(server_url);
      if (it == pub_video_urls_.end()) {
        if (multiurl_ && !topic_video_url_.empty()) {
          pub_video_urls_[server_url] = node_ns_->advertise<std_msgs::String>(topic_video_url_ + std::to_string((*itn)->getResourceId()), 1, true);
          ROS_INFO("IOP digital resource: advertising %s", pub_video_urls_[server_url].getTopic().c_str());
        }
      }
    }
  }

  void DigitalResourcePlugin::onIopVisualNames(IopVisualNamesPtr msg)
  {
    // update endpoints
    resource_names_.clear();
    for (size_t n = 0; n < msg->names.size(); n++) {
      unsigned short key = msg->names[n].resource_id;
      std::string name = msg->names[n].name;
      resource_names_[key] = name;
    }
    if (resource_names_.size() > 0) {
      std::vector<DigitalResourceCam*>::iterator it;
      for (it = cam_list_.begin(); it < cam_list_.end(); ++it) {
        (*it)->updateName(getResourceName((*it)->getResourceId()));
      }
    }
  }

  bool DigitalResourcePlugin::isIn(unsigned short resource_id, const fkie_iop_msgs::DigitalResourceEndpoints::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->endpoints.size(); i++) {
      if (msg->endpoints[i].resource_id == resource_id) {
        return true;
      }
    }
    return false;
  }

  bool DigitalResourcePlugin::isIn(unsigned short resource_id, std::vector<DigitalResourceCam*>& cam_list)
  {
    for (size_t i = 0; i < cam_list.size(); i++) {
      if (cam_list[i]->getResourceId() == resource_id) {
        return true;
      }
    }
    return false;
  }

  std::string DigitalResourcePlugin::getResourceName(unsigned short resource_id)
  {
    std::map<unsigned short, std::string>::iterator it = resource_names_.find(resource_id);
    if (it != resource_names_.end()) {
      return it->second;
    }
    return "";
  }

  void DigitalResourcePlugin::play(QString qurl, unsigned short resource_id)
  {
    std::string url = qurl.toStdString();
    ROS_INFO("IOP digital resource: play %s, resource id: %d", url.c_str(), resource_id);
    std::vector<DigitalResourceCam*>::iterator it;
    for (it = cam_list_.begin(); it < cam_list_.end(); ++it) {
      if (sender() != (*it) && ((*it)->getServerUrl() == url || multiurl_)) {
        (*it)->setSilentUnchecked(resource_id);
      }
    }
    if (!url.empty()) {
      std::map<std::string, ros::Publisher>::iterator itp = pub_video_urls_.find(url);
      if (itp != pub_video_urls_.end()) {
        std_msgs::String ros_msg_str;
        ros_msg_str.data = url;
        itp->second.publish(ros_msg_str);
      }
      played_urls_.push_back(url);
      std_msgs::UInt16 ros_msg;
      ros_msg.data = resource_id;
      pub_resource_id_.publish(ros_msg);
      std_msgs::String ros_msg_str;
      ros_msg_str.data = url;
      pub_video_url_.publish(ros_msg_str);
    } else {
      ROS_WARN("IOP digital resource: ignore play empty URL for resource id %d", resource_id);
    }
  }

  void DigitalResourcePlugin::stop(QString qurl, unsigned short resource_id)
  {
    std::string url = qurl.toStdString();
    ROS_INFO("IOP digital resource: stop %s, resource id: %d", url.c_str(), resource_id);
    if (!url.empty()) {
      std::map<std::string, ros::Publisher>::iterator itp = pub_video_urls_.find(url);
      if (itp != pub_video_urls_.end()) {
        std_msgs::String ros_msg_str;
        itp->second.publish(ros_msg_str);
      }
      std::vector<std::string>::iterator it = std::find(played_urls_.begin(), played_urls_.end(), url);
      if (it != played_urls_.end()) {
        played_urls_.erase(it);
      }
    }
    std_msgs::UInt16 ros_msg_id;
    ros_msg_id.data = 65535;
    pub_resource_id_.publish(ros_msg_id);
    std_msgs::String ros_msg_str;
    pub_video_url_.publish(ros_msg_str);
  }

  void DigitalResourcePlugin::showInfo()
  {
    if (cam_list_.size() > 0)
    {
      std::string info_msg("");
      for (size_t i = 0; i < cam_list_.size(); i++) {
          info_msg += "cam " + std::to_string(i+1) + ": " + cam_list_[i]->str() + "\n";
      }
      info_dialog_ = new QDialog(config_widget_);
      info_dialog_->setAttribute(Qt::WA_DeleteOnClose);
      info_dialog_->setMinimumWidth(150);
      QVBoxLayout* vlayout = new QVBoxLayout(info_dialog_);
      QLabel* info_label = new QLabel(QString::fromStdString(info_msg));
      info_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
      vlayout->addWidget(info_label);
      info_dialog_->setWindowTitle("cam info");
      info_dialog_->setWindowIcon(QIcon::fromTheme("help-about"));
      info_dialog_->show();
    }
  }

  void DigitalResourcePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["namespace"])
    {
      std::string ns;
      node["namespace"] >> ns;
      ui_.ns->setText(ns.c_str());
    }
    if (node["usemultiurl"])
    {
      bool usemultiurl;
      node["usemultiurl"] >> usemultiurl;
      ui_.cbMultiURL->setChecked(usemultiurl);
      multiurl_ = ui_.cbMultiURL->isChecked();
    }
    initTopics();
  }

  void DigitalResourcePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "namespace" << YAML::Value << ui_.ns->text().toStdString();
    emitter << YAML::Key << "usemultiurl" << YAML::Value << ui_.cbMultiURL->isChecked();
  }

  std::vector<std::string> DigitalResourcePlugin::split(const std::string& s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
  }
}
