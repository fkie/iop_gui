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

#include <fkie_iop_mapviz_plugins/digital_resource_cam.h>


namespace fkie_iop_mapviz_plugins
{
  DigitalResourceCam::DigitalResourceCam(fkie_iop_msgs::DigitalResourceEndpoint endpoint, std::string name) :
    QPushButton(QString::fromStdString(name)),
    name_(name),
    label_(""),
    server_type_(endpoint.server_type),
    server_url_(endpoint.server_url),
    resource_id_(endpoint.resource_id),
    address_(endpoint.address.subsystem_id, endpoint.address.node_id, endpoint.address.component_id),
    ignore_next_click_(false)
  {
    updateName(name);
    setCheckable(true);
    setFocusPolicy(Qt::NoFocus);
    setFixedHeight(24);
//      setFixedSize(57, 24);
    QObject::connect(this, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
  }

  DigitalResourceCam::~DigitalResourceCam()
  {
    ROS_INFO("delete digital resource cam %s", getServerUrl().c_str());
  }

  std::string DigitalResourceCam::str()
  {
    std::string result = server_url_ + " [ ";
    result += endpointTypeAsStr(server_type_) + " - ";
    result += std::to_string(resource_id_) + ": ";
    result += address_.str() + "]";
    return result;
  }

  void DigitalResourceCam::onToggled(bool checked)
  {
    if (checked) {
        if (!ignore_next_click_) {
            signalPlay(QString::fromStdString(getServerUrl()), getResourceId());
        }
        setStyleSheet("QPushButton { background-color: #98FB98;}");
    } else {
        if (!ignore_next_click_) {
            signalStop(QString::fromStdString(getServerUrl()), getResourceId());
        }
        setStyleSheet("QPushButton { background-color: None;}");
    }
    ignore_next_click_ = false;
  }

  void DigitalResourceCam::updateName(std::string name)
  {
    name_ = name;
    if (name.empty()) {
      name_ = endpointTypeAsStr(getServerType());
    }
    label_ = name_ + "/" + std::to_string(getResourceId());
    setText(QString::fromStdString(label_));
  }

  void DigitalResourceCam::setPlayed(bool state)
  {
    setChecked(state);
    if (state) {
      setStyleSheet("QPushButton { background-color: #98FB98;}");
    } else {
      setStyleSheet("QPushButton { background-color: None;}");
    }
  }

  void DigitalResourceCam::setSilentUnchecked(unsigned short ignore_id)
  {
    if (isChecked() && ignore_id != getResourceId()) {
      ignore_next_click_ = true;
      setChecked(false);
    }
  }

  std::string DigitalResourceCam::endpointTypeAsStr(int server_type)
  {
    if (server_type == 0) {
        return "RTSP";
    } else if (server_type == 1) {
        return "MPEG2TS";
    } else if (server_type == 2) {
        return "FTP";
    } else if (server_type == 3) {
        return "SFTP";
    } else if (server_type == 4) {
        return "FTP_SSH";
    } else if (server_type == 5) {
        return "HTTP";
    } else if (server_type == 6) {
        return "HTTPS";
    } else if (server_type == 7) {
        return "SCP";
    }
    return "";
  }

}
