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

#include <fkie_iop_mapviz_plugins/illuminator_plugin.h>
#include <fkie_iop_mapviz_plugins/illuminator_button.h>

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

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fkie_iop_mapviz_plugins::IlluminatorPlugin, mapviz::MapvizPlugin)


namespace fkie_iop_mapviz_plugins
{


  IlluminatorPlugin::IlluminatorPlugin() :
  mapviz::MapvizPlugin(),
    initialized_(false),
    config_widget_(new QWidget()),
    map_canvas_(NULL),
    ns_("/ocu/"),
    topic_name_states_("/ocu/illuminator_states"),
    node_(NULL)
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
    topics_.push_back("diagnostic_msgs/DiagnosticStatus");
    QObject::connect(this, &IlluminatorPlugin::VisibleChanged, this, &IlluminatorPlugin::setVisible);
    QObject::connect(ui_.select_states, &QPushButton::clicked, this, &IlluminatorPlugin::selectStates);
    QObject::connect(ui_.edit_states, &QLineEdit::editingFinished, this, &IlluminatorPlugin::statesEdited);
  }

  IlluminatorPlugin::~IlluminatorPlugin()
  {
    if (map_canvas_) {
      map_canvas_->removeEventFilter(this);
    }
    shutdownTopics();
    // if (timer_update_robots_) {
    //   QObject::disconnect(timer_update_robots_, SIGNAL(timeout()), this, SLOT(updateRobots()));
    //   timer_update_robots_->stop();
    // }
  }

  void IlluminatorPlugin::selectStates()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(topics_);
    if (topic.name.empty()) {
      return;
    }
    std::vector<std::string> tokens = split(topic.name, '/');
    std::string ns;
    for (size_t i = 0; i < tokens.size()-1; i++) {
      ROS_INFO("JOIN %s", tokens[i].c_str());
      ns += tokens[i] + "/";
    }
    ns_ = ns;
    ui_.edit_states->setText(QString::fromStdString(topic.name));
    statesEdited();
  }

  void IlluminatorPlugin::statesEdited()
  {
    std::string topic = ui_.edit_states->text().trimmed().toStdString();
    if (topic != topic_name_states_)
    {
      PrintWarning("No messages received.");
      initTopics();
      topic_name_states_ = topic;
    }
  }

  void IlluminatorPlugin::setVisible(bool visible)
  {
    if (visible) {
      map_canvas_->installEventFilter(this);
      initTopics();
    } else {
      map_canvas_->removeEventFilter(this);
      shutdownTopics();
    }
  }

  QWidget* IlluminatorPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool IlluminatorPlugin::Initialize(QGLWidget* canvas)
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

  void IlluminatorPlugin::initTopics()
  {
      if (topic_name_states_ != ui_.edit_states->text().toStdString())
      {
        topic_name_states_ = ui_.edit_states->text().toStdString();
        shutdownTopics();
      }
      if (node_ == NULL) {
        PrintError("no state messages received");
        node_ = new ros::NodeHandle();
        sub_states_ = node_->subscribe<diagnostic_msgs::DiagnosticStatus>(topic_name_states_, 100, &IlluminatorPlugin::callbackIlluminatorStates, this);
        ROS_INFO("IOP illuminator: subscribing to %s", sub_states_.getTopic().c_str());
      }
  }

  void IlluminatorPlugin::shutdownTopics()
  {
      clear_buttons();
      sub_states_.shutdown();
      if (node_ != NULL) {
        delete node_;
        node_ = NULL;
      }
  }

  void IlluminatorPlugin::callbackIlluminatorStates(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
  {
    clear_buttons();
    PrintInfo(std::to_string(msg->values.size()) + " illuminators discovered");
		for (size_t i = 0; i < msg->values.size(); i++) {
      diagnostic_msgs::KeyValue keyval = msg->values[i];
      IlluminatorButton* btn = new IlluminatorButton(keyval.key, ns_, keyval.value);
      ui_.buttonsFrame->layout()->addWidget(btn);
      //btn->show();
      //btn->size();
		}
    //ui_.buttonsFrame->layout()->update();
    //ui_.buttonsFrame->adjustSize();
    config_widget_->hide();
    config_widget_->show();
  }

  void IlluminatorPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic_states"])
    {
      std::string topic_states;
      node["topic_states"] >> topic_states;
      ui_.edit_states->setText(topic_states.c_str());
    }
    initTopics();
  }

  void IlluminatorPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic_states" << YAML::Value << ui_.edit_states->text().toStdString();
  }

  void IlluminatorPlugin::clear_buttons()
  {
    QLayoutItem *wItem;
    while ((wItem = ui_.buttonsFrame->layout()->takeAt(0)) != 0) {
      if (wItem->widget())
        wItem->widget()->setParent(NULL);
      delete wItem;
    }
  }

  std::vector<std::string> IlluminatorPlugin::split(const std::string& s, char delimiter)
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
