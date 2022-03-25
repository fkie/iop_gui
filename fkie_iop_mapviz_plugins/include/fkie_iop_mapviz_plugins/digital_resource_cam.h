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

#ifndef FKIE_IOP_MAPVIZ_PLUGINS_DIGITAL_RESOURCE_CAM_H_
#define FKIE_IOP_MAPVIZ_PLUGINS_DIGITAL_RESOURCE_CAM_H_

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

// IOP
#include <Transport/JausAddress.h>

// Messages
#include <fkie_iop_msgs/DigitalResourceEndpoint.h>
// Other Plugin-Files

namespace fkie_iop_mapviz_plugins
{
  class DigitalResourceCam : public QPushButton
  {
    Q_OBJECT

   public:
    int MAX_AGE = 30.0;

    DigitalResourceCam(fkie_iop_msgs::DigitalResourceEndpoint endpoint, std::string name);
    virtual ~DigitalResourceCam();

    std::string str();
    void updateName(std::string name);
    std::string endpointTypeAsStr(int server_type);
    unsigned char getServerType() { return server_type_; };
    std::string getServerUrl() { return server_url_; };
    unsigned short getResourceId() { return resource_id_; };
    JausAddress getAddress() { return address_; };
    void setPlayed(bool state);
    void setSilentUnchecked(unsigned short resource_id);

   Q_SIGNALS:
    void signalPlay(QString url, unsigned short resource_id);
    void signalStop(QString url, unsigned short resource_id);


   public Q_SLOTS:
    void onToggled(bool checked=false);

   private:
    std::string name_;
    std::string label_;
    unsigned char server_type_;
    std::string server_url_;
    unsigned short resource_id_;
    JausAddress address_;
    bool ignore_next_click_;
  };
}

#endif  // FKIE_IOP_MAPVIZ_PLUGINS_DIGITAL_RESOURCE_CAM_H_
