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
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <string>
#include <controllers/CamSetptCtrl.h>
#include <controllers/arm.h>
#include <rqt_quadcoptergui/QuadcopterInterfaceConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_QuadCopterwidget.h>

#include <parsernode/parser.h>

#include <boost/thread/mutex.hpp>

#include <pluginlib/class_loader.h>


#include <std_msgs/String.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

//#include <controllers/SetptCtrl.h>


#include <QCloseEvent>
#include <QCheckBox>
//#include <QMenuBar>
#include <QPushButton>
#include <QMessageBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QString>
#include <QTimer>
//#include <QMutex>
#include <boost/thread/mutex.hpp>
#include <QDockWidget>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <dynamixelsdk/arm_helper.h>

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

	//void StringCallback(const std_msgs::String::ConstPtr &data);

	ros::Subscriber vrpndata_sub;

	ros::Subscriber camdata_sub;

	ros::Subscriber joydata_sub;

	ros::Publisher desiredtraj_pub;


	//ros::Publisher stringdata_pub;

	virtual void shutdownPlugin();

	QString str;// Data to be put into the textbox;

	//QMutex qgui_mutex_;//Mutex for refreshing 
	//boost::mutex spin_mutex;//Mutex for ros spinning and Qt threads

	//bool refreshtext;

	//Parser Library to communicate with quadcopter
	boost::shared_ptr<pluginlib::ClassLoader<parsernode::Parser> > parser_loader;
	boost::shared_ptr<parsernode::Parser> parserinstance;

	char buffer[600];//buffer for creating Qstring data
	parsernode::common::quaddata data;//Quadcopter data from parser
	geometry_msgs::Quaternion goalposn;

	boost::shared_ptr<CameraSetptCtrl> ctrlrinst;
	boost::shared_ptr<gcop::Arm> arminst;
	boost::shared_ptr<dynamixelsdk::DynamixelArm> arm_hardwareinst;
	ros::Timer goaltimer;
	void cmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
	void camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);
	bool enable_joy;
	bool enable_camctrl;


	//Moving goal dynamically:
	tf::Vector3 diff_goal;
	tf::Vector3 diff_velgoal;
	tf::Vector3 curr_goal;
	tf::Vector3 bias_vrpn;
	tf::Vector3 vrpnrpy;//Latest vrpnrpy
	float bias_count;
	//The point is that if there is a bias in vrpn data then, the quadcopter will oscillate not about the actual position but around some angle which
	//is the  bias in vrpn

	void goaltimerCallback(const ros::TimerEvent&);
	int goalcount;
	float goalyaw;
	bool startcontrol, testctrlr;
	float corrected_thrustbias;

	bool followtraj;
	float traj_amp, traj_freq, traj_skew;

	ros::Time trajtime_offset;
	int joymsg_prevbutton, buttoncount;
	int joymsg_prevbutton1, buttoncount1;
	double as[2][3];//Arm inverse kinematics output
	double armlocaltarget[3];//Arm goal (Where the object is to grab)
	//double delta_armgoal[3];//Arm Goal in the Frame of the Quadcopter
	tf::Vector3 target;//Extraction target point
	double armpwm[3];//Arm pwm
	double armangles[3];//The angles of the arm in radians in gcop convention
	std::string uav_name;
	bool cam_partialcontrol;//Only use the object position to set the goal position 
	double yoffset_object;//Offset of the object in x direction 
	bool updategoal_dynreconfig;//Flag for updating the dynamic reconfigure goal parameters whenever it is triggered (This will write the values in dynreconfig instead of reading from it)
	double actual_armangles[2];//The angles obtained from dynamixelsdk
  double tip_position[3];//Tip Position

	//Reconfigure stuff:
	boost::shared_ptr<dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig> >reconfigserver;
	dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>::CallbackType reconfigcallbacktype;

	void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

	//Storing the current frame along with time stamp:
	tf::StampedTransform UV_O;
	//Storing the current object pose in Quadcopter frame
	tf::StampedTransform OBJ_QUAD_stamptransform;
	//Fixed Transforms  for converting Quad to Camera frame and object_mod transforms:
	tf::StampedTransform CAM_QUAD_transform;
	tf::StampedTransform OBJ_MOD_transform;

	tf::Vector3 errorrpy;

	//Storing the current command being set:
	geometry_msgs::Quaternion rescmdmsg;

	//Logger Stuff
	//ofstream cmdfile;
	ofstream vrpnfile;
	ofstream camfile;
	bool enable_logging;
	bool reconfiginit;
	int throttlecmdrate,ratecount;
	int armcmdrate,armratecount;
	tf::Vector3  object_armoffset;//Offset for object when arm has to catch it
	boost::shared_ptr<visualization_msgs::Marker> trajectoryPtr;
	boost::shared_ptr<visualization_msgs::Marker> targetPtr;
	//boost::shared_ptr<visualization_msgs::Marker> finaltipPtr; TODO add final tip frame to see
	tf::Transform quadtobase;
  boost::shared_ptr<tf::TransformBroadcaster> broadcaster;//Transform Broadcaster
	ros::Time start_grabbing;
	double timeout_grabbing;//Timeout for waiting to grab object usually a very short time to just stay for few seconds

	protected slots:
	virtual void wrappertakeoff();
	virtual void wrapperLand();
	virtual void wrapperDisarm();
	virtual void wrapperimu_recalib(int);
	virtual void follow_trajectory(int);
	virtual void integrator_control(int);
	virtual void enable_disablecontroller(int);
	virtual void enable_disablecamctrl(int);
	virtual void enable_disablelog(int);
	virtual void enable_disablemanualarmctrl(int);
	virtual void RefreshGui();
	//virtual void Capture_Target();
};

}
#endif // rqt_quadcopter_gui_H
