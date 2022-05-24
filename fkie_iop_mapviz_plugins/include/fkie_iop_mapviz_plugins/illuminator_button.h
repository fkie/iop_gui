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

#ifndef FKIE_IOP_MAPVIZ_PLUGINS_ILLUMINATOR_BUTTON_H_
#define FKIE_IOP_MAPVIZ_PLUGINS_ILLUMINATOR_BUTTON_H_

// C++ standard libraries
#include <map>
#include <string>
#include <vector>

// QT libraries
#include <QObject>
#include <QPushButton>
#include <QString>

// ROS libraries
#include <ros/ros.h>

// Messages
#include <std_msgs/Bool.h>


// Other Plugin-Files

namespace fkie_iop_mapviz_plugins
{
  class IlluminatorButton : public QPushButton
  {
    Q_OBJECT

   public:
    IlluminatorButton(std::string name, std::string ns, std::string state);
    virtual ~IlluminatorButton();

   public Q_SLOTS:
    void onToggled(bool checked=false);

   protected:
    void initTopics();
    void shutdownTopics();
    void callbackIlluminatorState(const std_msgs::Bool::ConstPtr& msg);

   private:
    std::string name_;
    std::string ns_;
    ros::NodeHandle node_;
    ros::Publisher pub_cmd_;
    ros::Subscriber sub_state_;
  };
}

#endif  // FKIE_IOP_MAPVIZ_PLUGINS_ILLUMINATOR_BUTTON_H_
