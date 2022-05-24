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

#include <fkie_iop_mapviz_plugins/illuminator_button.h>

#include <iostream>
#include <string>
#include <cstring>

namespace fkie_iop_mapviz_plugins
{
  IlluminatorButton::IlluminatorButton(std::string name, std::string ns, std::string state) :
    QPushButton(QString::fromStdString(name)),
    name_(name),
    ns_(ns),
    node_(ns + "/illuminator")
  {
    setCheckable(true);
    setFocusPolicy(Qt::NoFocus);
    setFixedHeight(24);
//      setFixedSize(57, 24);
    std::string on_state("on");
    bool checked = strncasecmp(state.c_str(), on_state.c_str(), 2) == 0;
    if (checked) {
        setStyleSheet("QPushButton { background-color: #98FB98;}");
    } else {
        setStyleSheet("QPushButton { background-color: None;}");
    }
    setChecked(checked);
    QObject::connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
    initTopics();
  }

  IlluminatorButton::~IlluminatorButton()
  {
    ROS_INFO("delete illuminator button %s%s", ns_.c_str(), name_.c_str());
  }

  void IlluminatorButton::onToggled(bool checked)
  {
    if (checked) {
        setStyleSheet("QPushButton { background-color: #98FB98;}");
    } else {
        setStyleSheet("QPushButton { background-color: None;}");
    }
    std_msgs::Bool rosmsg;
    rosmsg.data = checked;
    pub_cmd_.publish(rosmsg);
  }


  void IlluminatorButton::initTopics()
  {
      pub_cmd_ = node_.advertise<std_msgs::Bool>("cmd_" + name_, 1, true);
      ROS_INFO("IOP illuminator: advertising %s", pub_cmd_.getTopic().c_str());
      sub_state_ = node_.subscribe<std_msgs::Bool>(name_, 100, &IlluminatorButton::callbackIlluminatorState, this);
      ROS_INFO("IOP illuminator: subscribing to %s", sub_state_.getTopic().c_str());
  }

  void IlluminatorButton::shutdownTopics()
  {
      pub_cmd_.shutdown();
      sub_state_.shutdown();
  }

  void IlluminatorButton::callbackIlluminatorState(const std_msgs::Bool::ConstPtr& msg)
  {
    setChecked(msg->data);
  }

}
