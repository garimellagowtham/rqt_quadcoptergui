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
#include <controllers/SetptCtrl.h>
#include <rqt_quadcoptergui/QuadcopterInterfaceConfig.h>
#include <dynamic_reconfigure/server.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_quadcoptergui/ui_QuadCopterwidget.h>

#include "rqt_quadcoptergui/parser.h"

#include <boost/thread/mutex.hpp>

#include <pluginlib/class_loader.h>


#include <std_msgs/String.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

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
#define NSINES 10

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

	ros::Subscriber joydata_sub;

	//ros::Publisher stringdata_pub;

	virtual void shutdownPlugin();

	QString str;// Data to be put into the textbox;

	//QMutex qgui_mutex_;//Mutex for refreshing 
	//boost::mutex spin_mutex;//Mutex for ros spinning and Qt threads

	//bool refreshtext;

	//Parser Library to communicate with quadcopter
	boost::shared_ptr<pluginlib::ClassLoader<rqt_quadcoptergui::Parser> > parser_loader;
	boost::shared_ptr<rqt_quadcoptergui::Parser> parserinstance;

	char buffer[600];//buffer for creating Qstring data
	rqt_quadcoptergui::common::quaddata data;//Quadcopter data from parser
	geometry_msgs::Quaternion goalposn;

	boost::shared_ptr<SetptCtrl> ctrlrinst;
	ros::Timer goaltimer;
	void cmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);

	//Moving goal dynamically:
	tf::Vector3 diff_goal;
	tf::Vector3 diff_velgoal;
	tf::Vector3 curr_goal;
	void goaltimerCallback(const ros::TimerEvent&);
	int goalcount;
	float goalyaw;
	bool startcontrol, testctrlr;
	float corrected_thrustbias;
	
	bool perturbationon;
	int  perturb_axis;
	float perturb_amp, perturb_freq;

	float *frequencies;
	float *phases;
	float amplitude, attenuationcoeff;
	ros::Time perturbtime_offset;
	int joymsg_prevbutton, buttoncount;
	int joymsg_prevbutton1, buttoncount1;
	std::vector<float>armpwm;

	//Reconfigure stuff:
	 boost::shared_ptr<dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig> >reconfigserver;
	 dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>::CallbackType reconfigcallbacktype;

	 void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

	 //Storing the current frame along with time stamp:
	 tf::StampedTransform UV_O;

	 tf::Vector3 errorrpy;

	 //Storing the current command being set:
	 geometry_msgs::Quaternion rescmdmsg;

	 //Logger Stuff
	 //ofstream cmdfile;
	 ofstream vrpnfile;
	 bool enable_logging;
	 bool reconfiginit;
	 int throttlecmdrate,ratecount;

protected slots:
	virtual void wrappertakeoff();
	virtual void wrapperLand();
	virtual void wrapperDisarm();
	virtual void wrapperimu_recalib(int);
	virtual void perturb_control(int);
	virtual void integrator_control(int);
	virtual void enable_disablecontroller(int);
	virtual void enable_disablelog(int);
	virtual void wrapper_estthrustbias(int);
	virtual void RefreshGui();
};

}
#endif // rqt_quadcopter_gui_H
