/*
 * Copyright (c) 2011,  Gowtham Garimella JHU
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

#include <rqt_quadcoptergui/quadcoptergui.h>

#include <pluginlib/class_list_macros.h>

//#define ARM_ENABLED
//#define LOG_DEBUG

namespace rqt_quadcoptergui {

QuadcopterGui::QuadcopterGui() : rqt_gui_cpp::Plugin()
                                , context_(0)
                                , widget_(0)
                                , update_gui(false)
{
  setObjectName("QuadcopterGui");
}

QuadcopterGui::~QuadcopterGui()
{
}

void QuadcopterGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);


  context_ = &context;

  widget_->setWindowTitle("Quadcoptergui[*]");

  if (context.serialNumber() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }

  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);

  //set ui textbrowser to be read only:
  ui_.textBrowser->setReadOnly(true);

  //Get ros NodeHandle from parent nodelet manager:
  ros::NodeHandle nh = getNodeHandle();
  //Subscribers:
  gui_state_subscriber_ = nh.subscribe("/gui_state",10,&QuadcopterGui::guistateCallback, this);

  quad_state_subscriber_ = nh.subscribe("/quad_status",10,&QuadcopterGui::quadstateCallback,this);

  //Publishers:
  gui_command_publisher_ = nh.advertise<rqt_quadcoptergui::GuiCommandMessage>("/gui_commands",10);


  //Setup the timer to refresh Gui
  timer = new QTimer(widget_);
  connect(timer, SIGNAL(timeout()), this, SLOT(RefreshGui()));
  timer->start(50);//20Hz

  //Connect all the slots as needed
  connect(ui_.Takeoffbutton, SIGNAL(clicked()), this, SLOT(wrappertakeoff()));
  //connect(ui_.TargetCapturebutton, SIGNAL(clicked()), this, SLOT(Capture_Target()));
  connect(ui_.Landbutton, SIGNAL(clicked()), this, SLOT(wrapperLand()));
  connect(ui_.Disarmbutton, SIGNAL(clicked()), this, SLOT(wrapperDisarm()));
  connect(ui_.imucheckbox,SIGNAL(stateChanged(int)),this,SLOT(wrapperimu_recalib(int)));
  connect(ui_.follow_traj,SIGNAL(stateChanged(int)),this,SLOT(follow_trajectory(int)));
  connect(ui_.log_checkbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablelog(int)));
  connect(ui_.enable_joycheckbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablemanualarmctrl(int)));
  connect(ui_.integrator_checkbox,SIGNAL(stateChanged(int)),this,SLOT(integrator_control(int)));
  connect(ui_.enable_controller,SIGNAL(stateChanged(int)),this,SLOT(enable_disablecontroller(int)));
  connect(ui_.camcheckbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablecamctrl(int)));
}

void QuadcopterGui::RefreshGui()
{
  qgui_mutex_.lock();
  QString msg = QString::fromStdString(quad_status);
  qgui_mutex_.unlock();
  ui_.textBrowser->setPlainText(msg);
}


bool QuadcopterGui::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == widget_ && event->type() == QEvent::Close)
	{
		ROS_INFO("Closing...");
		event->ignore();
		context_->closePlugin();
		return true;
	}
	return QObject::eventFilter(watched, event);
}

///////////////////SLOTS//////////////////////

void QuadcopterGui::enable_disablelog(int state)//0
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_log;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::integrator_control(int state)//1
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_integrator;
  gui_command_publisher_.publish(msg);
}


void QuadcopterGui::follow_trajectory(int state)//2
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.traj_on;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::enable_disablecontroller(int state)//3
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_ctrl;
  gui_command_publisher_.publish(msg);
}
void QuadcopterGui::enable_disablemanualarmctrl(int state)//4
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_joy;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::enable_disablecamctrl(int state)//5
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_cam;
  gui_command_publisher_.publish(msg);
}
void QuadcopterGui::wrappertakeoff()//6
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  msg.commponent_name = msg.arm_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperLand()//7
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  msg.commponent_name = msg.land_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperDisarm()//8
{
  qgui_mutex_.lock();
  bool update_gui_ = update_gui;
  qgui_mutex_.unlock();
  if(update_gui_)//Do not run slot functionality if we are updating gui using onboard node's input
  {
    return;
  }
  GuiCommandMessage msg;
  msg.commponent_name = msg.disarm_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperimu_recalib(int state)
{
  ROS_WARN("Not Implemented");
}



void QuadcopterGui::shutdownPlugin()
{
  gui_state_subscriber_.shutdown();
  quad_state_subscriber_.shutdown();

  gui_command_publisher_.shutdown();
}

////////////////CALLBACKS///////////////

void QuadcopterGui::guistateCallback(const GuiStateMessage &statemsg)
{
  qgui_mutex_.lock();
  update_gui = true;
  qgui_mutex_.unlock();
  //Change UI states:
  switch(statemsg.commponent_id)
  {
  case statemsg.camera_controlstatus:
    ui_.camcheckbox->setCheckState(CHECKSTATE(statemsg.status));
    break;
  case statemsg.controller_status:
    ui_.enable_controller->setCheckState(CHECKSTATE(statemsg.status));
    break;
  case statemsg.integrator_status:
    ui_.integrator_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    break;
  case statemsg.joystick_status:
    ui_.enable_joycheckbox->setCheckState(CHECKSTATE(statemsg.status));
    break;
  case statemsg.log_status:
    ui_.log_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    break;
  case statemsg.trajectory_tracking_status:
    ui_.follow_traj->setCheckState(CHECKSTATE(statemsg.status));
    break;
  }
  qgui_mutex_.lock();
  update_gui = false;
  qgui_mutex_.unlock();
}

void QuadcopterGui::quadstateCallback(const std_msgs::String & statemsg)
{
  qgui_mutex_.lock();
  quad_status = statemsg.data;
  qgui_mutex_.unlock();
}

}
PLUGINLIB_DECLARE_CLASS(rqt_quadcoptergui, QuadcopterGui, rqt_quadcoptergui::QuadcopterGui, rqt_gui_cpp::Plugin)
