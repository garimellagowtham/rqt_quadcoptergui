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

  gcop_trajectory_publisher_ = nh.advertise<gcop_comm::CtrlTraj>("/mbsddp/traj_resp",10);

  //Create Visualizer:
  gcop_trajectory_visualizer_.reset(new GcopTrajectoryVisualizer(nh));

  //Setup the timer to refresh Gui
  timer = new QTimer(widget_);
  connect(timer, SIGNAL(timeout()), this, SLOT(RefreshGui()));
  timer->start(50);//20Hz

  //Connect all the slots as needed
  connect(ui_.Takeoffbutton, SIGNAL(clicked()), this, SLOT(wrappertakeoff()));
  //connect(ui_.TargetCapturebutton, SIGNAL(clicked()), this, SLOT(Capture_Target()));
  connect(ui_.Landbutton, SIGNAL(clicked()), this, SLOT(wrapperLand()));
  connect(ui_.LoadTrajectorybutton, SIGNAL(clicked()), this, SLOT(loadTrajectory()));
  connect(ui_.SendTrajectorybutton, SIGNAL(clicked()), this, SLOT(sendTrajectory()));
  connect(ui_.Disarmbutton, SIGNAL(clicked()), this, SLOT(wrapperDisarm()));
  //connect(ui_.imucheckbox,SIGNAL(stateChanged(int)),this,SLOT(wrapperimu_recalib(int)));
  connect(ui_.follow_traj,SIGNAL(stateChanged(int)),this,SLOT(follow_trajectory(int)));
  connect(ui_.log_checkbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablelog(int)));
  connect(ui_.enable_joycheckbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablemanualarmctrl(int)));
  connect(ui_.integrator_checkbox,SIGNAL(stateChanged(int)),this,SLOT(integrator_control(int)));
  connect(ui_.enable_controller,SIGNAL(stateChanged(int)),this,SLOT(enable_disablecontroller(int)));
  connect(ui_.camcheckbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablecamctrl(int)));
  connect(ui_.manualtargetcheckbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablemanualtargetretrieval(int)));
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
  if(checkUpdateState(0))//Check if input is from onboard Node
   return;
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
  if(checkUpdateState(1))//Check if input is from onboard Node
   return;
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
  if(checkUpdateState(2))//Check if input is from onboard Node
   return;

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
  if(checkUpdateState(3))//Check if input is from onboard Node
   return;
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
  if(checkUpdateState(4))//Check if input is from onboard Node
   return;
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
  if(checkUpdateState(5))//Check if input is from onboard Node
   return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.enable_cam;
  gui_command_publisher_.publish(msg);
}

void QuadcopterGui::enable_disablemanualtargetretrieval(int state)//6
{
  if(checkUpdateState(6))//Check if input is from onboard Node
   return;
  GuiCommandMessage msg;
  if(state == Qt::Checked)
    msg.command = true;
  else
    msg.command = false;
  msg.commponent_name = msg.manual_targetretrievalstatus;
  gui_command_publisher_.publish(msg);
}

////////////Buttons Callback Functions
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

void QuadcopterGui::loadTrajectory()
{
  //Load a trajectory from FileBox:
  trajectory_file_name = std::string();//Empty
  if(ros::param::has("/trajectory_file_name"))
    ros::param::get("/trajectory_file_name", trajectory_file_name);
  std::ifstream ifile;
  if(trajectory_file_name.empty())
  {
    QString filename = QFileDialog::getOpenFileName(widget_, "Open Trajectory File", "/home","All files (*.*)");
    trajectory_file_name = filename.toStdString();
  }
  ROS_INFO("Opening File: %s",trajectory_file_name.c_str());

  ifile.open(trajectory_file_name);

  if(!ifile.is_open())
  {
    std::cerr<<"Cannot Open File"<<trajectory_file_name<<std::endl;
    return;
  }
  //Ignore First line:
  ifile.ignore(1000,'\n');//Wait till new line
  //File should have t, traj_x,y,z, yaw; traj_vx,vy,vz in global frame;
  quadcopter_trajectory.reset(new gcop_comm::CtrlTraj());
  
  gcop_comm::State desired_state;
  geometry_msgs::Point pt;
  quadcopter_trajectory->N = -1;
  //Get Current Goal from reconfig params
  double xg = 0, yg = 0, zg = 0, yawg = 0;
  ros::param::get("/onboard_node/xg",xg);
  ros::param::get("/onboard_node/yg",yg);
  ros::param::get("/onboard_node/zg",zg);
  ros::param::get("/onboard_node/yawg",yawg);
  double data[8];
  while(1)
  {
    //While EOF has not been reached:
    ifile>>data[0]>>data[1]>>data[2]>>data[3]>>data[4]>>data[5]>>data[6]>>data[7];
    if(ifile.eof())//Break if EOF is reached
    {
      ROS_INFO("EOF Reached!");
      break;
    }
    //Printing File for DEBUG
    std::cout<<data[0]<<"\t"<<data[1]<<"\t"<<data[2]<<"\t"<<data[3]<<"\t"<<data[4]<<"\t"<<data[5]<<"\t"<<data[6]<<"\t"<<data[7]<<std::endl;
    quadcopter_trajectory->time.push_back(data[0]);
    quadcopter_trajectory->N++;
    //State:
    desired_state.basepose.rotation = tf::createQuaternionMsgFromYaw(data[4]+yawg);
    desired_state.basepose.translation.x = xg+data[1];
    desired_state.basepose.translation.y = yg+data[2];
    desired_state.basepose.translation.z = zg+data[3];
    desired_state.basetwist.linear.x = data[5];
    desired_state.basetwist.linear.y = data[6];
    desired_state.basetwist.linear.z = data[7];
    quadcopter_trajectory->statemsg.push_back(desired_state);
  }
  ifile.close();
  gcop_trajectory_visualizer_->publishTrajectory(*quadcopter_trajectory);
}
void QuadcopterGui::sendTrajectory()
{
  //To send quadcopter trajectory to Onboard Node:
  if(quadcopter_trajectory)
    gcop_trajectory_publisher_.publish(quadcopter_trajectory);
}
/*void QuadcopterGui::wrapperimu_recalib(int state)
{
  ROS_WARN("Not Implemented");
}
*/



void QuadcopterGui::shutdownPlugin()
{
  gui_state_subscriber_.shutdown();
  quad_state_subscriber_.shutdown();

  gui_command_publisher_.shutdown();
  gcop_trajectory_publisher_.shutdown();

  //Clear Variables:
  quadcopter_trajectory.reset();
  gcop_trajectory_visualizer_.reset();
  //delete update_component_id;
  //delete timer;
  //delete widget_;
}

////////////////CALLBACKS///////////////

void QuadcopterGui::guistateCallback(const GuiStateMessage &statemsg)
{
  //Change UI states:
  switch(statemsg.commponent_id)
  {
  case statemsg.camera_controlstatus:
    if(statemsg.status != ui_.camcheckbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.camcheckbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.controller_status:
    if(statemsg.status != ui_.enable_controller->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.enable_controller->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.integrator_status:
    if(statemsg.status != ui_.integrator_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.integrator_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.joystick_status:
    if(statemsg.status != ui_.enable_joycheckbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.enable_joycheckbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.log_status:
    if(statemsg.status != ui_.log_checkbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.log_checkbox->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.trajectory_tracking_status:
    if(statemsg.status != ui_.follow_traj->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.follow_traj->setCheckState(CHECKSTATE(statemsg.status));
    }
    break;
  case statemsg.manual_targetretrievalstatus:
    if(statemsg.status != ui_.manualtargetcheckbox->isChecked())
    {
      qgui_mutex_.lock();
      update_component_id[statemsg.commponent_id] = true;
      qgui_mutex_.unlock();
      ui_.manualtargetcheckbox->setCheckState(CHECKSTATE(statemsg.status));
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
