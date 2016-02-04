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
                                , update_component_id()
                                , trajectory_file_name()
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
  connect(ui_.Landbutton, SIGNAL(clicked()), this, SLOT(wrapperLand()));
  connect(ui_.Disarmbutton, SIGNAL(clicked()), this, SLOT(wrapperDisarm()));
  connect(ui_.InitiializeMPCButton, SIGNAL(clicked()), this, SLOT(wrapperInitializeMPC()));
  connect(ui_.tracking_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangeTracking(int)));
  connect(ui_.log_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangeLogging(int)));
  connect(ui_.velcontrol_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangeVelControl(int)));
  connect(ui_.rpycontrol_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangeRpyControl(int)));
  connect(ui_.poscontrol_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangePosControl(int)));
  connect(ui_.mpc_state_checkbox, SIGNAL(stateChanged(int)),SLOT(stateChangeMPCControl(int)));
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

////////////SLOT Functions
void QuadcopterGui::wrappertakeoff()
{
  GuiCommandMessage msg;
  msg.commponent_name = msg.arm_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperLand()
{
  GuiCommandMessage msg;
  msg.commponent_name = msg.land_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperDisarm()
{
  GuiCommandMessage msg;
  msg.commponent_name = msg.disarm_quad;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::wrapperInitializeMPC()
{
  GuiCommandMessage msg;
  msg.commponent_name = msg.initialize_mpc;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::RefreshGui()
{
  qgui_mutex_.lock();
  QString msg = QString::fromStdString(quad_status);
  qgui_mutex_.unlock();
  ui_.textBrowser->setPlainText(msg);
}

void QuadcopterGui::stateChangeLogging(int state)
{
  if(checkUpdateState(state_msg.log_status))//Check if input is from onboard Node
    return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_log;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::stateChangeVelControl(int state)
{
    if(checkUpdateState(state_msg.vel_control_status))//Check if input is from onboard node
        return;
    GuiCommandMessage msg;
    if(state == Qt::Checked)
        msg.command = true;
    else
        msg.command = false;
    msg.commponent_name = msg.enable_vel_control;
    gui_command_publisher_.publish(msg);
}

void QuadcopterGui::stateChangeRpyControl(int state)
{
    if(checkUpdateState(state_msg.rpyt_control_status))//Check if input is from onboard node
        return;
    GuiCommandMessage msg;
    if(state == Qt::Checked)
        msg.command = true;
    else
        msg.command = false;
    msg.commponent_name = msg.enable_rpyt_control;
    gui_command_publisher_.publish(msg);
}

void QuadcopterGui::stateChangePosControl(int state)
{
  if(checkUpdateState(state_msg.pos_control_status))//Check if input is from onboard node
    return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_pos_control;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::stateChangeMPCControl(int state)
{
  if(checkUpdateState(state_msg.mpc_control_status))//Check if input is from onboard node
    return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_mpc_control;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::stateChangeTracking(int state)
{
  if(checkUpdateState(state_msg.tracking_status))//Check if input is from onboard Node
    return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_tracking;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::shutdownPlugin()
{
  gui_state_subscriber_.shutdown();
  quad_state_subscriber_.shutdown();

  gui_command_publisher_.shutdown();
}

////////////////CALLBACKS///////////////

void QuadcopterGui::guistateCallback(const GuiStateMessage & statemsg)
{
  //Change UI states:
  switch(statemsg.commponent_id)
  {
  case statemsg.log_status:
    if(statemsg.status != ui_.log_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.log_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.tracking_status:
    if(statemsg.status != ui_.tracking_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.tracking_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.vel_control_status:
    if(statemsg.status != ui_.velcontrol_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.velcontrol_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.rpyt_control_status:
    if(statemsg.status != ui_.rpycontrol_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.rpycontrol_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.pos_control_status:
    if(statemsg.status != ui_.poscontrol_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.poscontrol_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.mpc_control_status:
    if(statemsg.status != ui_.mpc_state_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.poscontrol_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  }
}

void QuadcopterGui::quadstateCallback(const std_msgs::String & statemsg)
{
  qgui_mutex_.lock();
  quad_status = statemsg.data;
  qgui_mutex_.unlock();
}

}
PLUGINLIB_DECLARE_CLASS(rqt_quadcoptergui, QuadcopterGui, rqt_quadcoptergui::QuadcopterGui, rqt_gui_cpp::Plugin)
