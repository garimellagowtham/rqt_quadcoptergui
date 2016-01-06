/*
 * Copyright (c) 2011, Dorian Scholz, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_quadcopter_gui_H
#define rqt_quadcopter_gui_H

#include <string>
#include <fstream>
#include <rqt_gui_cpp/plugin.h>
#include <ui_QuadCopterwidget.h>

#include <boost/thread/mutex.hpp>

#include "ros/ros.h"

//TF helper functions:
#include <tf/transform_datatypes.h>


//Ros Messages
#include <std_msgs/String.h>
#include <rqt_quadcoptergui/GuiCommandMessage.h>
#include <rqt_quadcoptergui/GuiStateMessage.h>
#include <visualization_msgs/Marker.h>

#include <QCloseEvent>
#include <QCheckBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFileDialog>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QString>
#include <QTimer>
#include <QMutex>
#include <boost/thread/mutex.hpp>
#include <QDockWidget>


#define NOFJOINTS 2 //Arm joints

#define CHECKSTATE(input) input==true?Qt::Checked:Qt::Unchecked

namespace rqt_quadcoptergui {

class QuadcopterGui
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  QuadcopterGui();

  ~QuadcopterGui();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	virtual bool eventFilter(QObject* watched, QEvent* event);


protected:

  qt_gui_cpp::PluginContext* context_;

  Ui::QuadCopterwidget ui_;

  QTimer *timer;

  QWidget* widget_;

  std::string quad_status;// Data to be put into the textbox;

  //Subscribers:
  ros::Subscriber gui_state_subscriber_;

  ros::Subscriber quad_state_subscriber_;

  //Publisher:
  ros::Publisher gui_command_publisher_;

  ////Helper Variables
  QMutex qgui_mutex_;//Mutex for refreshing states
  bool update_component_id[9];// Only update gui and do not execute signal functions
  std::string trajectory_file_name;//Prespecified file name
  GuiStateMessage state_msg;//Dummy state message to get tags for enable_log etc


protected:
  virtual void shutdownPlugin();
  //////Helper Function:
  inline bool checkUpdateState(int index)
  {
    qgui_mutex_.lock();
    if(update_component_id[index])//Do not run slot functionality if we are updating gui using onboard node's input
    {
      update_component_id[index] = false;
      qgui_mutex_.unlock();
      return true;
    }
    qgui_mutex_.unlock();
    return false;
  }
  inline geometry_msgs::Point vec2Point(geometry_msgs::Vector3 vector)
  {
    geometry_msgs::Point pt;
    pt.x = vector.x;
    pt.y = vector.y;
    pt.z = vector.z;
  }
  //////Callbacks
  void guistateCallback(const rqt_quadcoptergui::GuiStateMessage&);

  void quadstateCallback(const std_msgs::String &);

protected slots:
  virtual void wrappertakeoff();
  virtual void wrapperLand();
  virtual void wrapperDisarm();
  virtual void RefreshGui();
  virtual void stateChangeTracking(int);
  virtual void stateChangeLogging(int);
  virtual void stateChangeVelControl(int);
  virtual void stateChangeRpyControl(int);
  virtual void stateChangePosControl(int);
};

}
#endif // rqt_quadcopter_gui_H
