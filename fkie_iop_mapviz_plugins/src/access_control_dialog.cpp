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

#include <fkie_iop_mapviz_plugins/access_control_dialog.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QLayoutItem>
#include <QObject>


namespace fkie_iop_mapviz_plugins
{
  AccessControlDialog::AccessControlDialog(const std::vector<AccessControlRobot *> robotlist):
  QDialog()
  {
    ui_.setupUi(this);
    update(robotlist);
  }

  AccessControlDialog::~AccessControlDialog()
  {
    ROS_INFO("delete access control dialog");
    removeRobotWidgets();
  }

  void AccessControlDialog::update(const std::vector<AccessControlRobot *> robotlist)
  {
    std::vector<AccessControlRobot *>::const_iterator it;
    for (it = robotlist.begin(); it != robotlist.end(); ++it) {
      bool added = false;
      for (int index = 0; index != ui_.layout_robots->count(); index++) {
        AccessControlRobot* irobot = dynamic_cast<AccessControlRobot*>(ui_.layout_robots->itemAt(index)->widget());
        if (irobot->getRobotUI().button_control->objectName() == QString((*it)->name().c_str())) {
          added = true;
          break;
        } else if (irobot->getRobotUI().button_control->objectName() > QString((*it)->name().c_str())) {
          ui_.layout_robots->insertWidget(index, *it);
          (*it)->setVisible(true);
          added = true;
          break;
        }
      }
      if (!added) {
        ui_.layout_robots->addWidget(*it);
        (*it)->setVisible(true);
      }
    }
  }

  void AccessControlDialog::closeEvent(QCloseEvent *e)
  {
    removeRobotWidgets();
    QDialog::closeEvent(e);
  }

  void AccessControlDialog::removeRobotWidgets()
  {
    QLayoutItem *child;
    while ((child = ui_.layout_robots->takeAt(0)) != 0)  {
        child->widget()->setParent(NULL);
        child->widget()->setVisible(false);
        delete child;
    }
  }

}
