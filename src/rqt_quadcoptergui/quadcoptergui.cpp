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

#include "quadcoptergui.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_quadcoptergui {

QuadcopterGui::QuadcopterGui() : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
	, goalcount(0),goalyaw(0)
	,startcontrol(false) ,testctrlr(true)
	, corrected_thrustbias(0)
	, enable_logging(false)
	, reconfiginit(false)
	, throttlecmdrate(1), ratecount(0)
	, perturbationon(false), perturb_amp(0), perturb_freq(0), perturb_axis(1)
	{
		setObjectName("QuadcopterGui");
		parser_loader.reset(new pluginlib::ClassLoader<rqt_quadcoptergui::Parser>("rqt_quadcoptergui","rqt_quadcoptergui::Parser"));
		UV_O.setIdentity();
		errorrpy.setValue(0,0,0);
		//frequencies = new float[NSINES]{0.0400,0.0600,0.1400,0.2200,0.4600,1.0601,2.0602,4.4604,9.5808,20.7818};
		//phases = new float[NSINES]{0.2760,0.6797,0.6551,0.1626,0.1190,0.4984,0.9597,0.3404,0.5853,0.2238};
		//amplitude = 29.0f/2582.0f;//the amplitude should be scaled down by (Number of sample in the window/2) otherwise there will be disasters :) The max perturbation comes out to 0.3183 using this
		amplitude = 50.0f/2582.0f;//the amplitude should be scaled down by (Number of sample in the window/2) otherwise there will be disasters :) The max perturbation comes out to 0.3183 using this
		attenuationcoeff = 0.06;//To decrease the amplitude of high frequency components in stimulus
		//frequencies = new float[NSINES]{0.0400,0.0600,   0.1000,   0.2200,   0.3400 ,  0.7401 ,  1.3401 ,  2.6202 ,  5.1404 , 10.4209};
		//phases = new float[NSINES]{ 0.0377,   0.8852,   0.9133,   0.7962,   0.0987,   0.2619,   0.3354,   0.6797,   0.1366,   0.7212};
		//Indices in FFT: 2           3           7          11          23          53         103         223         479        1039 Hopefully as this is not directly applied
		frequencies = new float[NSINES]{0.0400,   0.0600,   0.1000,   0.1400,   0.2200,   0.3400,   0.4600,   0.7401,   1.2201,   2.0202};
		phases = new float[NSINES]{0.0596,  0.6820,   0.0424,   0.0714,   0.5216,   0.0967,   0.8181,   0.8175,   0.7224,   0.1499};
		//Indices:  2     3     5     7    11    17    23    37    61   101

		//The sampling frequency is 103.289 and window freq = 0.020017 (Around 50 Sec)
	}

QuadcopterGui::~QuadcopterGui()
{
	delete frequencies;
	delete phases;
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

	//stringdata_pub = nh.advertise<std_msgs::String>("stringout",1);

	try
	{
		string parserplugin_name;
		if(!nh.getParam("/gui/parser_plugin",parserplugin_name))
		{
			ROS_ERROR("Cannot find parser_plugin parameter to load the parser");
			return;
		}
		parserinstance = parser_loader->createInstance(parserplugin_name);
		parserinstance->initialize(nh);
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

	//Connect all the slots as needed

	//Setup the timer:
	timer = new QTimer(widget_);
	connect(timer, SIGNAL(timeout()), this, SLOT(RefreshGui()));
	timer->start(50);//20Hz
	if(!nh.getParam("/gui/test_ctrlr",testctrlr))
	{
		testctrlr = true;
	}
	if(testctrlr)
		ROS_INFO("In Test Controller Mode");
	connect(ui_.Takeoffbutton, SIGNAL(clicked()), this, SLOT(wrappertakeoff()));
	connect(ui_.Landbutton, SIGNAL(clicked()), this, SLOT(wrapperLand()));
	connect(ui_.Disarmbutton, SIGNAL(clicked()), this, SLOT(wrapperDisarm()));
	connect(ui_.imucheckbox,SIGNAL(stateChanged(int)),this,SLOT(wrapperimu_recalib(int)));
	connect(ui_.perturb_checkbox,SIGNAL(stateChanged(int)),this,SLOT(perturb_control(int)));
	connect(ui_.log_checkbox,SIGNAL(stateChanged(int)),this,SLOT(enable_disablelog(int)));
	connect(ui_.integrator_checkbox,SIGNAL(stateChanged(int)),this,SLOT(integrator_control(int)));
	connect(ui_.thrust_estcheckbox,SIGNAL(stateChanged(int)),this,SLOT(wrapper_estthrustbias(int)));
	connect(ui_.enable_controller,SIGNAL(stateChanged(int)),this,SLOT(enable_disablecontroller(int)));
	//connect(ui_.ctrlcheckbox,SIGNAL(stateChanged(int)),this,SLOT(switchctrlr(int)));

	//Create a controller instance from the controllers library:
	//Get the parameter to know whether to test the controller or directly use it. In testing mode, the thrust value is set to be a very small value and roll and pitch are normal. This way you can move and see if it is doing what its supposed to do
	ctrlrinst.reset(new SetptCtrl(nh));
	parserinstance->getquaddata(data);
	ctrlrinst->setextbias(data.thrustbias); //Fext initial guess comes from the parser. We will need to estimate it for some quadcopters if its used in commanding it.

	// Logger
	string logdir = "/home/gowtham";//Default name
	if(!nh.getParam("/gui/logdir",logdir))
	{
		ROS_WARN("Cannot find log directory");
	}
	else
	{
		logdir = logdir + "/session";
		string logdir_stamped = rqt_quadcoptergui::common::addtimestring(logdir);
		ROS_INFO("Creating Log dir: %s",logdir_stamped.c_str());
		mkdir(logdir_stamped.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);//Create the directory see http://pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
		vrpnfile.open((logdir_stamped+"/vrpn.dat").c_str());//TODO add warning if we cannot open the file
		vrpnfile.precision(9);
		vrpnfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z \t Quat.X \t Quat.Y \t Quat.Z \t Quat.W"<<endl;
		//cmdfile.open(logdir_stamped+"/cmd.dat");//TODO add warning if we cannot open the file
    parserinstance->setlogdir(logdir_stamped);
    ctrlrinst->setlogdir(logdir_stamped);
	}
	
	//subscribe to vrpndata for now
	string uav_posename;
	if(!nh.getParam("/gui/vrpn_pose",uav_posename))
	{
		ROS_ERROR("Cannot load uav pose parameter");
		return;
	}
	vrpndata_sub = nh.subscribe(uav_posename,1,&QuadcopterGui::cmdCallback,this);

	//Connect to dynamic reconfigure server:
	reconfigserver.reset(new dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>(nh));
  reconfigcallbacktype = boost::bind(&QuadcopterGui::paramreqCallback, this, _1, _2);
  reconfigserver->setCallback(reconfigcallbacktype);
	//Create timer for moving Goal Dynamically:
	goaltimer = nh.createTimer(ros::Duration(0.02), &QuadcopterGui::goaltimerCallback,this);//50Hz So the goal can go upto 25 Hz  Nyquist rate
	goaltimer.stop();
}

void QuadcopterGui::RefreshGui()
{
	if(!parserinstance)
	{
		ROS_ERROR("No parser instance created");
		return;
	}
	parserinstance->getquaddata(data);
	Matrix3x3 rotmat = UV_O.getBasis();
	tf::Vector3 vrpnrpy;
	rotmat.getEulerYPR(vrpnrpy[2],vrpnrpy[1],vrpnrpy[0]);
	errorrpy = vrpnrpy - tf::Vector3(data.rpydata.x, data.rpydata.y,data.rpydata.z);
	//errorrpy.setValue(0,0,0);
	tf::Vector3 quadorigin = UV_O.getOrigin();
	// Create a Text message based on the data from the Parser class
	sprintf(buffer,
			"Battery Percent: %2.2f\t\nTemperature: %2.2f\tPressure: %2.2f\tWindspeed: %2.2f\tAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\nMagx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\nposx: %2.2f\tposy: %2.2f\tposz: %2.2f\nvrpnr: %2.2f\tvrpnp: %2.2f\tvrpny: %2.2f\nErrorr: %2.2f\tErrorrp: %2.2f\tErrory: %2.2f\nresr: %2.2f\tresp: %2.2f\tresy: %2.2f\trest: %2.2f\nMass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s", 
			data.batterypercent
			,data.temperature,data.pressure
			,data.wind_speed, data.altitude
			,data.rpydata.x*(180/M_PI),data.rpydata.y*(180/M_PI),data.rpydata.z*(180/M_PI)
			,data.magdata.x,data.magdata.y,data.magdata.z
			,data.linacc.x,data.linacc.y,data.linacc.z
			,data.linvel.x,data.linvel.y,data.linvel.z
			,quadorigin[0], quadorigin[1], quadorigin[2]
			,vrpnrpy[0]*(180/M_PI),vrpnrpy[1]*(180/M_PI),vrpnrpy[2]*(180/M_PI)
			,errorrpy[0]*(180/M_PI),errorrpy[1]*(180/M_PI),errorrpy[2]*(180/M_PI)
			, (rescmdmsg.x)*(180/M_PI), (rescmdmsg.y)*(180/M_PI), rescmdmsg.z, rescmdmsg.w
			,data.mass,data.timestamp,data.quadstate.c_str());
			//, (rescmdmsg.x-data.rpydata.x)*(180/M_PI), (rescmdmsg.y-data.rpydata.y)*(180/M_PI), rescmdmsg.z*(180/M_PI), rescmdmsg.w

	//Also  add status about the vrpn data and controller errors etc

	QString msg = QString::fromStdString(buffer);
	ui_.textBrowser->setPlainText(msg);
}

/*void QuadcopterGui::StringCallback(const std_msgs::String::ConstPtr &stringdata)
{
	ROS_INFO("Hello");
	qgui_mutex_.lock();
	str =  QString::fromStdString(stringdata->data);
	qgui_mutex_.unlock();
	std_msgs::String stringpubdata;
	stringpubdata.data = stringdata->data;
	stringdata_pub.publish(stringpubdata);
	refreshtext = true;
}
*/


bool QuadcopterGui::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
		ROS_INFO("Closing...");
    event->ignore();
    context_->closePlugin();
		//stringdata_sub.shutdown();
    return true;
  }
	return QObject::eventFilter(watched, event);
}

void QuadcopterGui::wrappertakeoff()
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot take off");
		return;
	}
	//Set the extbias back to nominal value
	ctrlrinst->setextbias(data.thrustbias); //Set the external force back to nominal value Just extra safety its already set in check_control etc
	parserinstance->takeoff();
}

void QuadcopterGui::wrapperLand()
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot take off");
		return;
	}
	startcontrol = false;//Stop the controller
	parserinstance->land();
	rescmdmsg.x = 0;rescmdmsg.y = 0; rescmdmsg.z = 0; rescmdmsg.w = 0;//reset command
	//Uncheck the controller checkbox to be consistent:
	ui_.enable_controller->setCheckState(Qt::Unchecked);
}

void QuadcopterGui::wrapperDisarm()
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot take off");
		return;
	}
	startcontrol = false;//Stop the controller
	rescmdmsg.x = 0;rescmdmsg.y = 0; rescmdmsg.z = 0; rescmdmsg.w = 0;//reset command
	parserinstance->disarm();
	//Uncheck the controller checkbox to be consistent:
	ui_.enable_controller->setCheckState(Qt::Unchecked);
}

void QuadcopterGui::wrapper_estthrustbias(int state)
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot take off");
		return;
	}
	if(state == Qt::Checked)
	{
		parserinstance->estimatethrustbias();
		parserinstance->getquaddata(data);
		ctrlrinst->setextbias(data.thrustbias); //Set the new update thrustbias
		ui_.thrust_estcheckbox->setCheckState(Qt::Unchecked);
	}
}

void QuadcopterGui::enable_disablelog(int state)
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot Log");
		return;
	}
	if(!ctrlrinst)
	{
		ROS_WARN("Controller Instance not defined. Cannot Log");
		return;
	}
	if(state == Qt::Checked)
	{
		if(enable_logging == false)
		{
			ROS_INFO("I am called");
			enable_logging = true;
			parserinstance->controllog(true);
			ctrlrinst->controllog(true);
		}
	}
	else
	{
		if(enable_logging == true)
		{
			ROS_INFO("I am called2");
			enable_logging = false;
			parserinstance->controllog(false);
			ctrlrinst->controllog(false);
		}
	}
}

void QuadcopterGui::enable_disablecontroller(int state)
{
	//Set the extbias back to nominal value
	ctrlrinst->setextbias(data.thrustbias); //Set the external force back to nominal value
	if(state == Qt::Checked)
	{
		ctrlrinst->integratethrust(true);
		ui_.integrator_checkbox->setCheckState(Qt::Checked);
		ui_.log_checkbox->setCheckState(Qt::Checked);
		startcontrol = true;
		goaltimer.start();
	}
	else
	{
		ctrlrinst->integratethrust(false);
		startcontrol = false;
		ui_.integrator_checkbox->setCheckState(Qt::Unchecked);
		goaltimer.stop();
	}
}

void QuadcopterGui::wrapperimu_recalib(int state)
{
	if(!parserinstance)
	{
		ROS_WARN("Parser Instance not defined. Cannot take off");
		return;
	}
	if(state == Qt::Checked)
	{
		parserinstance->calibrateimubias();
		ui_.imucheckbox->setCheckState(Qt::Unchecked);
	}
}

void QuadcopterGui::integrator_control(int state)
{
	if(!ctrlrinst)
	{
		ROS_WARN("Ctrl inst not defined");
	}
	if(state == Qt::Checked)
	{
		ctrlrinst->integratethrust(true);
	}
	else if(state == Qt::Unchecked)
	{
		ctrlrinst->integratethrust(false);
	}
}

void QuadcopterGui::perturb_control(int state)
{
	if(state == Qt::Checked)
	{
		perturbationon = true;
	}
	else if(state == Qt::Unchecked)
	{
		perturbationon = false;
		goalcount = 1;
		diff_goal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
		diff_velgoal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
	}
}

void QuadcopterGui::shutdownPlugin()
{
	parserinstance.reset();
	parser_loader.reset();
	ctrlrinst.reset();
	vrpndata_sub.shutdown();
	reconfigserver.reset();
	goaltimer.stop();
	vrpnfile.close();//Close the file
	//cmdfile.close();//Close the file
	  //stringdata_sub.shutdown();
	  //stringdata_pub.shutdown();
}

void QuadcopterGui::cmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)
{
	/*	if(data.quadstate != "Flying")
			{
			transformStampedMsgToTF(*currframe,UV_O);//converts to the right format 
			return;
			}
	 */

	//Call the ctrlr to set the ctrl and then send the command to the quadparser
	if(!ctrlrinst)
	{
		ROS_WARN("Controller not instantiated");
		return;
	}
	controllers::ctrl_command rescmd;
	transformStampedMsgToTF(*currframe,UV_O);//converts to the right format 
	//Store the current position of the quadcopter for display
	//ctrlrinst->Set(UV_O, rescmd);
	ctrlrinst->Set(UV_O, errorrpy, rescmd);
	if(!parserinstance)
	{
		ROS_WARN("Parser not instantiated");
		return;
	}
	rescmdmsg.x = rescmd.roll; rescmdmsg.y = rescmd.pitch; rescmdmsg.z = rescmd.rateyaw; rescmdmsg.w = rescmd.thrust;
	if(testctrlr)
	{
		rescmdmsg.w = 0.2*data.thrustbias;
	}
	if(data.armed && startcontrol)
	{
		if((++ratecount) == throttlecmdrate)
		{
			ratecount = 0;
			parserinstance->cmdrpythrust(rescmdmsg,true);//Also controlling yaw
			//ROS_INFO("Setting cmd");
		}
	}
	if(enable_logging)
	{
		//Logging save to file
		vrpnfile<<(UV_O.stamp_.toNSec())<<"\t"<<(currframe->transform.translation.x)<<"\t"<<(currframe->transform.translation.y)<<"\t"<<(currframe->transform.translation.z)<<"\t"<<(currframe->transform.rotation.x)<<"\t"<<(currframe->transform.rotation.y)<<"\t"<<(currframe->transform.rotation.z)<<"\t"<<(currframe->transform.rotation.w)<<endl;
	}	
	if(!data.armed)//Once the quadcopter is armed we do not set the goal position to quad's origin, the user will set the goal. But the goal will not move until u set the enable_control The user should not give random goal once it is initialized.
	{
			curr_goal = UV_O.getOrigin();//set the current goal to be same as the quadcopter origin we dont care abt the orientation as of now
		  ctrlrinst->setgoal(curr_goal[0],curr_goal[1],curr_goal[2],goalyaw);//Set the goal to be same as the current position of the quadcopter the velgoal is by default 0
	}
	
	//Set the altitude of the quadcopter in the data
	parserinstance->setaltitude(currframe->transform.translation.z);
	//#ifdef PRINT
	//ROS_INFO("Rescmd: %f\t%f\t%f",rescmdmsg.x,rescmdmsg.y,rescmdmsg.w);
	//#endif
}

void QuadcopterGui::paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level)
{
	//ROS_INFO("Request recvd");
	// Use the config values to set the goals and gains for quadcopter
	if(!parserinstance || !ctrlrinst)
	{
		ROS_WARN("Parser or ctrlr not defined");
		return;
	}
	if(!reconfiginit)
	{
		//Get parameters
		ros::param::get("/ctrlr/kpr",config.kpr);
		ros::param::get("/ctrlr/kdr",config.kdr);
		ros::param::get("/ctrlr/kpt",config.kpt);
		ros::param::get("/ctrlr/kdt",config.kdt);
		ros::param::get("/ctrlr/throtbound",config.throtbound);
		ros::param::get("/ctrlr/rpbound",config.rpbound);
		ros::param::get("/ctrlr/cmdrate_throttle",config.cmdrate_throttle);
		ros::param::get("/ctrlr/perturb_amp",config.perturb_amp);
		ros::param::get("/ctrlr/perturb_freq",config.perturb_freq);
		ros::param::get("/ctrlr/perturb_axis",config.perturb_axis);
		reconfiginit = true;
	}
	if(level&0x0002)
	{
		tf::Vector3 quadorigin = UV_O.getOrigin();
		goalyaw = config.yawg;
		//Should not put anything else in level 2 #IMPORTANT
		if(data.armed)
		{
			goalcount = config.goalT*(5);//Just a hack to ensure the total time is ok TODO
			diff_goal.setValue((-curr_goal[0] + config.xg)/goalcount, (-curr_goal[1] + config.yg)/goalcount,(-curr_goal[2] + config.zg)/goalcount);
			diff_velgoal.setValue(0,0,0);
			//goaltimer.start();
		}
		else
		{
			config.xg = quadorigin[0];
			config.yg = quadorigin[1];
			config.zg = quadorigin[2];
			curr_goal = quadorigin;
			//goaltimer.start();
			//diff_goal.setValue(0, 0, (-curr_goal[2] + config.zg)/goalcount);
		}

	}
	ctrlrinst->setgains(config.kpr, config.kdr, config.kpt, config.kdt, config.kit);
	ctrlrinst->setbounds(config.throtbound, (M_PI/180.0)*config.rpbound);//This throttle bound is different from throttle bias which needs to be estimated for some quadcopters. This just cuts off the throttle values that are beyond thrustbias +/- throtbound

	throttlecmdrate = config.cmdrate_throttle;

	//setting perturbation parameters
	perturb_freq = config.perturb_freq;
	perturb_amp = config.perturb_amp;
	perturb_axis = config.perturb_axis;
	//corrected_thrustbias = config.add_thrustbias + data.thrustbias;
	//ctrlrinst->setextbias(corrected_thrustbias);//Set the corrected thrustbias
}

//Not much load to run this timer
void QuadcopterGui::goaltimerCallback(const ros::TimerEvent &event)
{
	//if(data.quadstate == "Flying")
	//{
	if(!perturbationon)
	{
		if(goalcount > 0)
		{
			curr_goal = curr_goal + diff_goal;
			cout<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<endl;
			if(!ctrlrinst)
			{
				ROS_WARN("Controller not instantiated");
				return;
			}
			ctrlrinst->setgoal(curr_goal[0],curr_goal[1],curr_goal[2],goalyaw);//By default the vel = 0;
			goalcount--;
		}
		perturbtime_offset = ros::Time::now();
	}
	else
	{ 
		ros::Duration currduration = ros::Time::now() - perturbtime_offset;
		float signal = 0, signalder = 0;
		for(int count1 = 0;count1 < NSINES;count1++)
		{
			signal += amplitude*(1/pow(frequencies[count1]/50,attenuationcoeff))*sin(2*M_PI*(frequencies[count1]*currduration.toSec()+phases[count1]));
			signalder += 2*M_PI*(1/pow(frequencies[count1]/50,attenuationcoeff))*frequencies[count1]*amplitude*cos(2*M_PI*(frequencies[count1]*currduration.toSec()+phases[count1]));
		}
		//cout<<"Signal: "<<signal<<"\t Signalder: "<<signalder<<endl;
		switch(perturb_axis)
		{
			case 1:
				//Adding sum of sines input given that the single sing input is done
				diff_goal.setValue(signal, 0, 0);
				diff_velgoal.setValue(signalder,0,0);
				break;
			case 2:
				diff_goal.setValue(0,signal,0);
				diff_velgoal.setValue(0,signalder,0);
				break;
			case 3:
				diff_goal.setValue(0,0,signal);
				diff_velgoal.setValue(0,0,signalder);
				break;
		}
		tf::Vector3 final_goal = curr_goal + diff_goal;
		ctrlrinst->setgoal(final_goal[0],final_goal[1],final_goal[2],goalyaw,diff_velgoal[0], diff_velgoal[1], diff_velgoal[2]);
	}
	/*
		 else
		 {
		 goaltimer.stop();
		 }
	 */
	//}
}
}
PLUGINLIB_DECLARE_CLASS(rqt_quadcoptergui, QuadcopterGui, rqt_quadcoptergui::QuadcopterGui, rqt_gui_cpp::Plugin)
