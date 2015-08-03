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
	, goalcount(0), reset_imu_count(0), goalyaw(0)
	, startcontrol(false) ,testctrlr(true)
	, corrected_thrustbias(0)
	, enable_logging(false)
	, reconfiginit(false)
	, throttlecmdrate(1), ratecount(0)
	, enable_joy(true), enable_camctrl(false)
	, armcmdrate(4), armratecount(0)
	, followtraj(false), traj_amp(0), traj_freq(0), traj_skew(1)
	, joymsg_prevbutton(0), buttoncount(0)
	, joymsg_prevbutton1(0), buttoncount1(0)
	, updategoal_dynreconfig(false), cam_partialcontrol(true), gripped_already(false)
	, broadcaster(new tf::TransformBroadcaster())
	, timeout_grabbing(3) , targetPtr(new visualization_msgs::Marker()) //, trajectoryPtr(new visualization_msgs::Marker())
	, waitingfortrajectory(true), openloop_mode(true),initialitrq(true) //By default we use openloop mode
	, newcamdata(false)
	, nearest_index_gcoptime(0)
	{
		setObjectName("QuadcopterGui");
		parser_loader.reset(new pluginlib::ClassLoader<parsernode::Parser>("parsernode","parsernode::Parser"));
		UV_O.setIdentity();
		errorrpy.setValue(0,0,0);
		target.setValue(0,0,0);//Initializing the target extraction point
		//quadtobase.setIdentity();
		//quadtobase.setOrigin(tf::Vector3(0.0732,0,-0.07));//The z distance needs to adjusted exactly

		//For now default value of object_offset:
		//object_armoffset = tf::Vector3(0,0.08,-0.18);//relative to the markers in Optitrack frame //For full camera control this SHOULD BE IN Object/Inertial Frame
		//object_armoffset = tf::Vector3(0,0.05,-0.25);//relative to the markers in Optitrack frame //For full camera control this SHOULD BE IN Object/Inertial Frame
		object_armoffset = tf::Vector3(0,0.0,-0.2);//relative to the markers in Optitrack frame //For full camera control this SHOULD BE IN Object/Inertial Frame
		//-0.07 was prev guess
		quadoffset_object = tf::Vector3(0, -0.63, 0.05);//Where the quadcopter should stay relative to the markers This is manually adjusted based on the accuracy of the quadcopter
		//in z dirxn have to see if 0.0 is ok or use -0.05
		//manual_offset = tf::Vector3(0,0.1,0);//This is the bias in estimation of the object. We have to find an automatic way of finding this

		//For now fixed value later will set this as a parameter:
		arm_basewrtquad.setValue(0.0732, 0, -0.1);

		// Prepare Iteration request:
		itrq.x0.statevector.resize(2*NOFJOINTS);
		itrq.xf.statevector.resize(2*NOFJOINTS);
		itrq.xf.basetwist.linear.x = 0; itrq.xf.basetwist.linear.y = 0; itrq.xf.basetwist.linear.z = 0; 
		itrq.xf.basetwist.angular.x = 0; itrq.xf.basetwist.angular.y = 0; itrq.xf.basetwist.angular.z = 0;

		targetPtr->id = 1;
		targetPtr->ns = "targetpickup";
		targetPtr->header.frame_id = "/optitrak";
		targetPtr->action = visualization_msgs::Marker::ADD;
		targetPtr->pose.orientation.w = 1.0;
		targetPtr->type = visualization_msgs::Marker::CUBE;
		targetPtr->scale.x = 0.04;
		targetPtr->scale.y = 0.1;
		targetPtr->scale.z = 0.1;
		targetPtr->color.r = 1.0;
		targetPtr->color.a = 1.0;

/*
					trajectoryPtr->id = 1;
					trajectoryPtr->points.resize(31);//Just fixed number of points in trajectoryPtr
					trajectoryPtr->header.frame_id = "/optitrak";
					trajectoryPtr->ns = "desired_traj";
					trajectoryPtr->action = visualization_msgs::Marker::ADD;
					trajectoryPtr->pose.orientation.w = 1.0;
					trajectoryPtr->id = 1;
					trajectoryPtr->type = visualization_msgs::Marker::LINE_STRIP;
					trajectoryPtr->scale.x = 0.05;//Need thick line
					trajectoryPtr->color.b = 1.0;
					trajectoryPtr->color.a = 1.0;
		 */
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
//	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle nh = getMTNodeHandle();

	//stringdata_pub = nh.advertise<std_msgs::String>("stringout",1);

	//Load Target:
	nh.getParam("/ctrlr/targetx",target[0]);
	nh.getParam("/ctrlr/targety",target[1]);
	nh.getParam("/ctrlr/targetz",target[2]);
	nh.getParam("/ctrlr/partialcam_control",cam_partialcontrol);
#ifdef ARM_ENABLED
	nh.getParam("/ctrlr/timeout_grabbing",timeout_grabbing);
#else
  timeout_grabbing = 1.0;// Hardset timeout of 1 sec
#endif
	nh.getParam("/ctrlr/openloop_mode",openloop_mode);

	//Load UAV Name:  Used in setting tf frame id
	nh.getParam("/gui/uav_name",uav_name);
	//Get The static transform for Camera to Quadcopter and similarly for object mod:
	//Setting Camera in Quad frame
	static tf::TransformListener listener;//Will make it a class member later (for other functions to use TODO)
	try{
		bool result = listener.waitForTransform(uav_name, "camera",
				ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform(uav_name, "camera",
				ros::Time(0), CAM_QUAD_transform);
		if(!result)
			cout<<"Cannot find QUAD to CAM Transform"<<endl;
		//Look for object modification transform
		result = listener.waitForTransform("object", "object_mod",
				ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("object", "object_mod",
				ros::Time(0), OBJ_MOD_transform);
		if(!result)
			cout<<"Cannot find OBJ_MOD Transform"<<endl;
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	/*
		 CAM_QUAD_transform.setOrigin(tf::Vector3(0.1,0, -0.05));
		 tf::Quaternion cam_quad_quat;
		 cam_quad_quat.setEulerZYX(M_PI,-M_PI/2,0);
		 CAM_QUAD_transform.setRotation(cam_quad_quat);
	//Setting OBJ_MOD transform:
	OBJ_MOD_transform.setIdentity();
	tf::Quaternion obj_mod_quat;
	obj_mod_quat.setEulerZYX(M_PI,-M_PI/2,0);
	OBJ_MOD_transform.setRotation(obj_mod_quat);
	 */
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
		return;
	}

	//Set gripper state to neutral in the beginning:
	parserinstance->grip(0);

	//Connect all the slots as needed

	//Setup the timer:
	timer = new QTimer(widget_);
	connect(timer, SIGNAL(timeout()), this, SLOT(RefreshGui()));
	timer->start(100);//10Hz
	if(!nh.getParam("/gui/test_ctrlr",testctrlr))
	{
		testctrlr = true;
	}
	if(testctrlr)
		ROS_INFO("In Test Controller Mode");
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
	//connect(ui_.ctrlcheckbox,SIGNAL(stateChanged(int)),this,SLOT(switchctrlr(int)));

	//Create a controller instance from the controllers library:
	//Get the parameter to know whether to test the controller or directly use it. In testing mode, the thrust value is set to be a very small value and roll and pitch are normal. This way you can move and see if it is doing what its supposed to do
	//ctrlrinst.reset(new CameraSetptCtrl(nh, broadcaster));
	ctrlrinst.reset(new SetptCtrl(nh, broadcaster));
	arminst.reset(new gcop::Arm);
	int dyn_deviceInd = 0;//Defaults
	int dyn_baudnum = 57600;
	nh.getParam("/dynamixel/deviceIndex",dyn_deviceInd);
	nh.getParam("/dynamixel/baudrate",dyn_baudnum);
#ifdef ARM_ENABLED
	arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
#endif
	arminst->l1 = 0.175;
	//arminst->l2 = 0.35;
	arminst->l2 = 0.42;
	arminst->x1 = 0.025;//Need to change this after measuring again TODO
	parserinstance->getquaddata(data);
	if(ctrlrinst)
	{
		double xbias, ybias, rateyawbias;
		nh.getParam("/bias_vrpnx",xbias);
		nh.getParam("/bias_vrpny",ybias);
		nh.getParam("/bias_rateyaw",rateyawbias);
		ctrlrinst->setextbias(data.thrustbias, xbias, ybias, rateyawbias);
		ROS_INFO("X,Y, RateYaw Bias: %f\t%f\t%f",xbias, ybias, rateyawbias);
		//ctrlrinst->setextbias(data.thrustbias); //Fext initial guess comes from the parser. We will need to estimate it for some quadcopters if its used in commanding it.
	}
	//bias_vrpn.setValue(2.33*(M_PI/180),-0.3*(M_PI/180),0);
	bias_vrpn.setValue(0,0,0);
	nh.getParam("/bias_vrpnroll",bias_vrpn[0]);
	nh.getParam("/bias_vrpnpitch",bias_vrpn[1]);
	nh.getParam("/bias_vrpnyaw",bias_vrpn[2]);
	bias_vrpn = (M_PI/180.0)*bias_vrpn;//Convert degrees to radians
	bias_count = 200;//Initial bias so we dont vary mean very much

	// Logger
	string logdir = "/home/gowtham";//Default name
	if(!nh.getParam("/gui/logdir",logdir))
	{
		ROS_WARN("Cannot find log directory");
	}
	else
	{
		logdir = logdir + "/session";
		string logdir_stamped = parsernode::common::addtimestring(logdir);
		ROS_INFO("Creating Log dir: %s",logdir_stamped.c_str());
		mkdir(logdir_stamped.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);//Create the directory see http://pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
		vrpnfile.open((logdir_stamped+"/vrpn.dat").c_str());//TODO add warning if we cannot open the file
		vrpnfile.precision(9);
		vrpnfile<<"#Time\tPos.X\tPos.Y\tPos.Z\tQuat.X\tQuat.Y\tQuat.Z\tQuat.W\tBias_R\tBias_P\tCurr_goal_x\tCurr_goal_y\tCurr_goal_z"<<endl;
		//Camfile
		camfile.open((logdir_stamped+"/campose.dat").c_str());//TODO add warning if we cannot open the file
		camfile.precision(9);
		camfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z \t Quat.X \t Quat.Y \t Quat.Z \t Quat.W\t Curr_goal.x\t Curr_goal.y\t Curr_goal.z\t Obj_origin.x\t Obj_origin.y\t Obj_origin.z"<<endl;
		//Arm Tip file
		tipfile.open((logdir_stamped+"/tippos.dat").c_str());//TODO add warning if we cannot open the file
		tipfile.precision(9);
		tipfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z\t DesPos.X\t DesPos.Y\t DesPos.Z\t Act_armangle0\t Act_armangle1\t Des_armangle0\t Des_armangle1"<<endl;
		//cmdfile.open(logdir_stamped+"/cmd.dat");//TODO add warning if we cannot open the file
		parserinstance->setlogdir(logdir_stamped);
		ctrlrinst->setlogdir(logdir_stamped);
	}

	//Prepare joinstate msg:
	jointstate_msg.header.frame_id = "/movingrobot/baselink";
	jointstate_msg.position.resize(NOFJOINTS);
	jointstate_msg.name.push_back("airbasetolink1");
	jointstate_msg.name.push_back("link1tolink2");//Initialization

	//subscribe to vrpndata now
	string uav_posename;
	if(!nh.getParam("/gui/vrpn_pose",uav_posename))
	{
		ROS_ERROR("Cannot load uav pose parameter");
		return;
	}
	cout<<"Subscribing to uav pose topic on: "<<uav_posename<<endl;
	vrpndata_sub = nh.subscribe(uav_posename,1,&QuadcopterGui::vrpnCallback,this);
	camdata_sub = nh.subscribe("/Pose_Est/objpose",1,&QuadcopterGui::camcmdCallback,this);
	joydata_sub = nh.subscribe("/joy",1,&QuadcopterGui::joyCallback,this);
	gcoptraj_sub = nh.subscribe("/mbsddp/traj_resp",1,&QuadcopterGui::gcoptrajectoryCallback,this);

	armtarget_pub = nh.advertise<visualization_msgs::Marker>("armtarget", 1);
	iterationreq_pub = nh.advertise<gcop_comm::Iteration_req>("/mbsddp/iteration_req",1);
	jointstate_pub = nh.advertise<sensor_msgs::JointState>("/movingrobot/joint_states",1);


	//Connect to dynamic reconfigure server:
	reconfigserver.reset(new dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>(nh));
	reconfigcallbacktype = boost::bind(&QuadcopterGui::paramreqCallback, this, _1, _2);
	reconfigserver->setCallback(reconfigcallbacktype);
	//Create timer for moving Goal Dynamically:
	goaltimer = nh.createTimer(ros::Duration(0.02), &QuadcopterGui::goaltimerCallback,this);//50Hz So the goal can go upto 25 Hz  Nyquist rate
	goaltimer.stop();
	timer_grabbing = nh.createTimer(ros::Duration(4), &QuadcopterGui::ClosingafterGrabbing, this, true);//One shot timer
	timer_grabbing.stop();
	timer_relaxgrip = nh.createTimer(ros::Duration(4), &QuadcopterGui::Oneshotgrabbing, this, true);//One shot timer
	timer_relaxgrip.stop();
	cmdtimer = nh.createTimer(ros::Duration(0.02), &QuadcopterGui::cmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
	cmdtimer.start();
}

void QuadcopterGui::RefreshGui()
{
	if(!parserinstance)
	{
		ROS_ERROR("No parser instance created");
		return;
	}
	//parserinstance->grip(-1);//Open arm [DEBUG]
	parserinstance->getquaddata(data);
	
	//Reset attitude on IMU every 10 Hz
	if(++reset_imu_count == 2)
	{
    static tf::Vector3 imu_vrpndiff((vrpnrpy[0] - data.rpydata.x),(vrpnrpy[1] - data.rpydata.y),(vrpnrpy[2] - data.rpydata.z));
		reset_imu_count = 0;
		if(parserinstance)
		{
			parserinstance->reset_attitude(vrpnrpy[0]-imu_vrpndiff[0], vrpnrpy[1]-imu_vrpndiff[1], vrpnrpy[2]-imu_vrpndiff[2]);
		}
	}
	//cout<<"Bias :"<<bias_vrpn[0]<<"\t"<<bias_vrpn[1]<<"\t"<<bias_vrpn[2]<<"\t"<<endl;
	//errorrpy.setValue(0,0,0);
	tf::Vector3 quadorigin = UV_O.getOrigin();
	tf::Vector3 obj_origin = OBJ_QUAD_stamptransform.getOrigin();
//	if(ui_.bias_estcheckbox->isChecked())
//	{
//		bias_vrpn[0] += -0.0005*(quadorigin[1] - curr_goal[1]);
//		bias_vrpn[1] += 0.0005*(quadorigin[0] - curr_goal[0]);
		/*bias_vrpn += (1/(bias_count+1))*(vrpnrpy - bias_vrpn);
		bias_vrpn[2] = 0;//Set Vrpn yaw bias to 0
		bias_count += 1;//Increase the count
		*/
//	}
	// Create a Text message based on the data from the Parser class
	sprintf(buffer,
			"Battery Percent: %2.2f\t\nTemperature: %2.2f\tPressure: %2.2f\tWindspeed: %2.2f\tAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\nMagx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\nposx: %2.2f\tposy: %2.2f\tposz: %2.2f\nvrpnr: %2.2f\tvrpnp: %2.2f\tvrpny: %2.2f\nErrorr: %2.2f\tErrorrp: %2.2f\tErrory: %2.2f\nresr: %2.2f\tresp: %2.2f\tresy: %2.2f\trest: %2.2f\nbias_roll: %2.2f\tbias_pitch: %2.2f\tbias_yaw: %2.2f\nObjx: %2.2f\tObjy: %2.2f\tObjz: %2.2f\t\nTipx: %2.2f\tTipy: %2.2f\tTipz: %2.2f\t\nMass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s", 
			data.batterypercent
			,data.temperature,data.pressure
			,data.wind_speed, data.altitude
			,data.rpydata.x*(180/M_PI),data.rpydata.y*(180/M_PI),data.rpydata.z*(180/M_PI)//IMU rpy angles
			,data.magdata.x,data.magdata.y,data.magdata.z
			,data.linacc.x,data.linacc.y,data.linacc.z
			,data.linvel.x,data.linvel.y,data.linvel.z
			,quadorigin[0], quadorigin[1], quadorigin[2]
			,vrpnrpy[0]*(180/M_PI),vrpnrpy[1]*(180/M_PI),vrpnrpy[2]*(180/M_PI)
			,errorrpy[0]*(180/M_PI),errorrpy[1]*(180/M_PI),errorrpy[2]*(180/M_PI)
			,(rescmdmsg.x)*(180/M_PI), (rescmdmsg.y)*(180/M_PI), rescmdmsg.z, rescmdmsg.w
			,bias_vrpn[0]*(180/M_PI),bias_vrpn[1]*(180/M_PI),bias_vrpn[2]*(180/M_PI)
			,obj_origin[0], obj_origin[1], obj_origin[2]
			,tip_position[0], tip_position[1], tip_position[2]
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

/*void QuadcopterGui::wrapper_estthrustbias(int state)
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
	}
	}
 */
void QuadcopterGui::enable_disablemanualarmctrl(int state) //This is also useful for folding the arm before landing
{
	//Get ros NodeHandle from parent nodelet manager:
	ros::NodeHandle nh = getMTNodeHandle();
	joymsg_prevbutton = 0; buttoncount = 0;
	joymsg_prevbutton1 = 0; buttoncount1 = 0;
	//cout<<"________________________I Am called _____________________"<<endl;
	if(state == Qt::Checked)
	{
		enable_joy = true;
	}
	else
	{
		enable_joy = false;
#ifndef LOG_DEBUG
		start_grabbing = ros::Time::now();//When we enter reachable workspace and disable joy then this should dictates when to start counting timeout for grabbing target
#endif
	}
#ifdef ARM_ENABLED
  if(arm_hardwareinst && parserinstance)
	{
		arm_hardwareinst->foldarm();//Just fold the arm whenever you switch between two modes Robustness

		//if(enable_joy) [WILL ENABLE IF NECESSARY]
			//parserinstance->grip(0);

		//if(!enable_joy)
		//	parserinstance->grip(-1);//Open gripper position in automatic position so that it will grip it automatically
	}
#endif
}
void QuadcopterGui::enable_disablecamctrl(int state) //To specify which controller to use either the camera one or the motion capture one
{
	//Testing only Optitrack
	if(!ctrlrinst)
	{
		ROS_WARN("Controller not instantiated");
		return;
	}
	if(state == Qt::Checked)
	{
		//[DEBUG]
		//cout<<"Ros Time: "<<ros::Time::now()<<"\t UV_O Time: "<<UV_O.stamp_<<endl;
		enable_camctrl = true;
		//Open the gripper :
		if(parserinstance)
		{
			parserinstance->grip(-1);//Open We changed open to be more energy efficient by increasing pwm width 
			timer_relaxgrip.setPeriod(ros::Duration(1));//2 seconds
			timer_relaxgrip.start();//Start oneshot timer;
		}
		if(cam_partialcontrol)//If partial control to debug we have to see the output of the goal whenever the quad is started up
		{
			goaltimer.start();//Redundancy
		}
		else
		{
			//[In full camera control since we still need arm control need to device a way to not stop goaltimer but set goalcount to 0 TODO]
			goaltimer.stop();//in full cam ctrl we do not use goal timer, instead the goal is directly set by object controller 
			//Specify the goal as origin (i.e the object itself is the goal)
			ctrlrinst->setgoal(0,0,0,0);//Set the goal to be same as the object the velgoal is by default 0
		}
	}
	else
	{
		cout<<"Disabling_Camctrl"<<endl;
		/////////////////////////This cannot be used as a fallback without optitrack system //////////////
		enable_camctrl = false;
		followtraj = false;
		waitingfortrajectory = true;//Resetting trajectory wait after one trial in open/closedloop mode
		initialitrq = true;//Reset initialization after one trial
		//Fold arm:
		//if(arm_hardwareinst)
			//arm_hardwareinst->foldarm();
		//Also set ui state of follow traj to false:
		ui_.follow_traj->setCheckState(Qt::Unchecked);

		//Also specify the goal as the current quad postion TODO
		//curr_goal = UV_O.getOrigin();//set the current goal to be same as the quadcopter origin we dont care abt the orientation as of now
		//tf::Vector3 centergoal(0.75, 0.9, 1.4);//Center of workspace with same height
		tf::Vector3 centergoal(0.67, 0.9, curr_goal[2]);//Center of workspace with same height
		updategoal_dynreconfig = true;//Set the flag to make sure dynamic reconfigure reads the new goal
		goalcount = 20; //Set the goal back to the specified posn smoothly
		diff_goal.setValue((-curr_goal[0] + centergoal[0])/goalcount, (-curr_goal[1] + centergoal[1])/goalcount,(-curr_goal[2] + centergoal[2])/goalcount);
		goaltimer.start();//Redundancy
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
		//ctrlrinst->integratethrust(true); Already done in integrator_checkbox
		ui_.integrator_checkbox->setCheckState(Qt::Checked);
		ui_.log_checkbox->setCheckState(Qt::Checked);
		startcontrol = true;
		goaltimer.start();
	}
	else
	{
		//ctrlrinst->integratethrust(false);
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
		return;
	}
	if(testctrlr)//If testing the controller, do not integrate the thrust 
	{
		ctrlrinst->integrate(false);//Redundancy 
		return;
	}
	if(state == Qt::Checked)
	{
		ctrlrinst->integrate(true);
	}
	else if(state == Qt::Unchecked)
	{
		ctrlrinst->integrate(false);
	}
}

void QuadcopterGui::follow_trajectory(int state)
{
	if(state == Qt::Checked)
	{
		followtraj = true;
		//[DEBUG]
		ROS_INFO("Following traj true");
	}
	else if(state == Qt::Unchecked)
	{
		followtraj = false;

		tf::Vector3 centergoal(0.67, 0.9, curr_goal[2]);//Center of workspace with same height
    ///////////CHANGE CENTER OF WORKSPACE TO BE A PARAMETER ///////////////////
		updategoal_dynreconfig = true;//Set the flag to make sure dynamic reconfigure reads the new goal
		goalcount = 20; //Set the goal back to the specified posn smoothly
		diff_goal.setValue((-curr_goal[0] + centergoal[0])/goalcount, (-curr_goal[1] + centergoal[1])/goalcount,(-curr_goal[2] + centergoal[2])/goalcount);
		goaltimer.start();//Redundancy

		//goalcount = 1;
		//diff_goal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
	}
}

void QuadcopterGui::shutdownPlugin()
{
	//Poweroff arm:
#ifdef ARM_ENABLED
	arm_hardwareinst->powermotors(false);
#endif
	parserinstance.reset();
	parser_loader.reset();
	ctrlrinst.reset();
	arminst.reset();
#ifdef ARM_ENABLED
	arm_hardwareinst.reset();
#endif

	vrpndata_sub.shutdown();
	camdata_sub.shutdown();
	joydata_sub.shutdown();
	gcoptraj_sub.shutdown();
	iterationreq_pub.shutdown();

	reconfigserver.reset();
	goaltimer.stop();
	cmdtimer.stop();
	vrpnfile.close();//Close the file
	camfile.close();//Close the file
	tipfile.close();//Close the file
	//trajectoryPtr.reset();
	targetPtr.reset();
	armtarget_pub.shutdown();
	//cmdfile.close();//Close the file
	//stringdata_sub.shutdown();
	//stringdata_pub.shutdown();
}

void QuadcopterGui::gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg)
{
  if(!followtraj)//If follow trajectory has not been enabled do not receive a trajectory
    return;
	//[DEBUG]
	ROS_INFO("Received Gcop Trajectory");
	gcop_trajectory = traj_msg;//Default Copy constructor (Instead if needed can write our own copy constructor)
	//Copying ourselves:
	//gcop_trajectory.N = traj_msg.N;
	//gcop_trajectory.time = traj_msg.time;
	//gcop_trajectory.
	waitingfortrajectory = false;
	nearest_index_gcoptime = 0;//Every time we get a new trajectory we reset the time index for searching nearest current time to zero
	if(openloop_mode)//In Closed loop we usually have fewer number of iterations vs in openloop where we do lots of iterations. Thus by the time we receive the traj we have some offset which we remove
	{
		request_time = ros::Time::now();
	}
}

void QuadcopterGui::joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg)
{
	if(!parserinstance)
	{
		ROS_WARN("Parser not instantiated");
		return;
	}
	//cout<<"Arm pwm: "<<joymsg->axes[0]<<"\t"<<joymsg->axes[1]<<"\t"<<joymsg->axes[2]<<"\t"<<joymsg->axes[3]<<"\t"<<endl;
	if((joymsg->buttons[0] - joymsg_prevbutton) > 0)
	{
		buttoncount = (buttoncount+1)%3;
		cout<<buttoncount<<endl;
	}
	joymsg_prevbutton = joymsg->buttons[0];
	if(buttoncount == 0)//Three state gripper
		parserinstance->grip(0);//Neutral
	else if(buttoncount == 1)
	{
		parserinstance->grip(1);//Hold
		//timer_relaxgrip.setPeriod(ros::Duration(2));//2 seconds
		//timer_relaxgrip.start();//Start oneshot timer;
	}
	else if(buttoncount == 2)
	{
		parserinstance->grip(-1);//Release
		timer_relaxgrip.setPeriod(ros::Duration(1));//2 seconds
		timer_relaxgrip.start();//Start oneshot timer;
	}
	//For starting to move arm:

	if(enable_joy)//If the joystick is not enable return
	{
		if((joymsg->buttons[1] - joymsg_prevbutton1) > 0)
		{
			buttoncount1 = (buttoncount1+1)%2;
			cout<<buttoncount1<<endl;
		}
		joymsg_prevbutton1 = joymsg->buttons[1];
	  double armpwm[2];//Arm pwm used by joystick
		if(buttoncount1 == 1)
		{
			armpwm[0] = joymsg->axes[1];
			armpwm[1] = joymsg->axes[0];//Will convert them also into angles later
#ifdef ARM_ENABLED
			if(arm_hardwareinst)
				arm_hardwareinst->setarmpwm(armpwm);
#endif
			//cout<<"Settting armpwm"<<endl;
		}
		else
		{
#ifdef ARM_ENABLED
			if(arm_hardwareinst)
				arm_hardwareinst->foldarm();
#endif
		}
	}
}


void QuadcopterGui::ClosingafterGrabbing(const ros::TimerEvent &event)
{
	ROS_INFO("Closing the arm and grabbing target");

	//if(followtraj)//If Followtraj, then this function is called exactly at the end of the trajectory
	//	parserinstance->grip(1);//Parser does not control arm directly anymore it only controls gripper
#ifdef ARM_ENABLED
  gripped_already = false;//Reset gripped already to false after one trial

#ifdef LOG_DEBUG
	if(arm_hardwareinst && parserinstance)
	{
		arm_hardwareinst->foldarm();
		parserinstance->grip(0);//Neutral We need to pull the object away
	}
#endif

#endif
	//Disabling Cam checkbox here
	ui_.camcheckbox->setCheckState(Qt::Unchecked);
}

void QuadcopterGui::Oneshotgrabbing(const ros::TimerEvent &event)
{
	parserinstance->grip(0);//Relax the grip
	ROS_INFO("Relaxing Grip");
}


//Camera callback listens to pose of object in Camera frame
void QuadcopterGui::camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)
{
	//static tf::TransformBroadcaster br;
	//cout<<"Cam called"<<endl;

	//Find the object pose in Quadcopter frame:
	tf::Transform OBJ_CAM_transform;
	transformMsgToTF(currframe->transform,OBJ_CAM_transform);//converts to the right format 
	tf::Transform OBJ_QUAD_transform = CAM_QUAD_transform*OBJ_CAM_transform*OBJ_MOD_transform;
	//+ quatRotate(UV_O.getRotation(),manual_offset) //Will incorporate it later
	OBJ_QUAD_stamptransform  = tf::StampedTransform(OBJ_QUAD_transform, currframe->header.stamp,uav_name,"object");//CAM_QUAD is the pose of Camera in Quadcopter frame, OBJ_MOD is the transform needed to make the object parallel (in terms of roll, pitch) with the Inertial frame

	if(!enable_camctrl)//Not Using Camera Control [VERY IMPORTANT] [Bad Coding fix TODO]
	{
		return;
	}


	//[DEBUG]
	//cout<<"Entering camcmdCallback"<<endl;
	if(!ctrlrinst)
	{
		ROS_WARN("Controller not instantiated");
		return;
	}




	tf::Vector3 object_origin;//Convenience variable for logging in future we will also use full camera ctrl and change how logging works

	//If using partialcontrol should not directly control quadcopter instead just set the goal in optitrack frame and use the optitrack controller to do the job:
	if(cam_partialcontrol)
	{
		//Set the goal based on OBJ_QUAD transform 
		tf::Transform OBJ_OPTITRACK_transform = UV_O*OBJ_QUAD_stamptransform;
		object_origin = OBJ_OPTITRACK_transform.getOrigin();
		tf::Vector3 OBJ_QUAD_origin = OBJ_QUAD_stamptransform.getOrigin();
		tf::Vector3 OBJ_QUAD_origin_inoptitrackframe = quatRotate(UV_O.getRotation(),OBJ_QUAD_origin);//Get the object in quadcopters frame  written in optirack frame

		tf::Vector3 ARMTarget_OPTITRACK_position = object_origin + object_armoffset; 

		targetPtr->pose.position.x = ARMTarget_OPTITRACK_position[0];//Target Object Position
		targetPtr->pose.position.y = ARMTarget_OPTITRACK_position[1];
		targetPtr->pose.position.z = ARMTarget_OPTITRACK_position[2];
	  armtarget_pub.publish(targetPtr);

		//Substract y offset (Assuming the object is to be approached in y dirxn) TODO Use object pose or some input dirxn of approach for grasping later//
		tf::Vector3 offset_quadposn = object_origin + quadoffset_object;
		if(followtraj)
		{
			//We are following gcop trajectory Checkout openloop case i.e only call iterationreq once with target number of iterations and try to follow the trajectory
			//Fill iteration request with all the data needed for doing optimization
			if((!(openloop_mode ||waitingfortrajectory)) || (openloop_mode && initialitrq) || (!openloop_mode && initialitrq)) //This logic is derived from truth table
			{
				//Fill itrq with data needed
				//[DEBUG]
				ROS_INFO("Entering Iteration req");
				if(!ctrlrinst)
				{
					ROS_WARN("Controller Instance not defined");
					return;
				}
				//Initial Condition:
				transformTFToMsg(ctrlrinst->UV_O_filt,itrq.x0.basepose);//converts to the right format  Basepose
				if(initialitrq)//Initially assume quadcopter is stable even though it has different angle TODO change this to add initialization in the request itself 
				{
					itrq.x0.basepose.rotation = tf::createQuaternionMsgFromYaw(M_PI/2);//Initial rotation is taken from knowledge
				}
				if(!initialitrq) //When initializing, we assume the quad is at zero velocity. This removes any small velocity that quad may have
					vector3TFToMsg((ctrlrinst->UV_O_filt)*filt_vel , itrq.x0.basetwist.linear); //Need to find Body Fixed angular velocity TODO
				for(int count1 = 0;count1 < 2*NOFJOINTS; count1++) //Joint angles and vel
					itrq.x0.statevector[count1] = actual_armstate[count1];

				//Goal Condition:
				//Move this out of camcmdCallback TODO
				//vector3TFToMsg(offset_quadposn, itrq.xf.basepose.translation);//Set the posn and Rotation is identity always
				itrq.xf.basepose.translation.x = 0.65;
				//itrq.xf.basepose.translation.y = 1.3;
				itrq.xf.basepose.translation.y = 1.15;
				itrq.xf.basepose.translation.z =  1.91;

				tf::Quaternion finalorientation = tf::createQuaternionFromYaw(M_PI/2);//Final yaw  Ideally this should be obtained from object pose TODO
				quaternionTFToMsg(finalorientation, itrq.xf.basepose.rotation);
				//Find final posn angles based on offset quad posn:
				tf::Vector3 offsetquadlocaltarget = (object_armoffset) - quadoffset_object;
				//Rotate with the inverse of final transformation 
				offsetquadlocaltarget = quatRotate(finalorientation.inverse(),offsetquadlocaltarget) - arm_basewrtquad;
				cout<<"Offset quad localtarget: "<<offsetquadlocaltarget[0]<<"\t"<<offsetquadlocaltarget[1]<<"\t"<<offsetquadlocaltarget[2]<<"\t"<<endl;
				//armlocaltarget[0] = offsetquadlocaltarget[0]; armlocaltarget[1] = offsetquadlocaltarget[1]; armlocaltarget[2] = offsetquadlocaltarget[2];
				armlocaltarget[0] = 0.52; armlocaltarget[1] = 0; armlocaltarget[2] = -0.05;
				//This offset is done in local frame which does not make sense always have to see what this amounts to
				double armres = arminst->Ik(as,armlocaltarget);
				cout<<as[0][0]<<"\t"<<as[0][1]<<"\t"<<as[0][2]<<"\t"<<as[1][0]<<"\t"<<as[1][1]<<"\t"<<as[1][2]<<endl;
				//cout<<"arm res: "<<armres<<endl;
				ROS_INFO("Arm Res: %f",armres);


				for(int count1 = 0;count1 < NOFJOINTS; count1++) //Final Joint angles and vel
				{

					itrq.xf.statevector[count1] = as[1][count1+1];//lower elbow solution
					itrq.xf.statevector[NOFJOINTS + count1] = 0;//Final commanded velocity
				}

				//Number of segments to return:
				itrq.N = 0;//If 0 it will provide all the segments
				itrq.tf = 3;//Final time to reach goal is 3 seconds TODO (For closed loop adjust this based on how we are progressing on the trajectory)

				iterationreq_pub.publish(itrq);
				request_time = (ctrlrinst->UV_O_filt).stamp_;//When the quad stamp was made //Later will add Getcurrent state from ctrlr through kalman filter for accurate estimation of current pose
				waitingfortrajectory = true;//Once published we are waiting for gcop to produce a trajectory
				initialitrq = false;//Once initialized, this is set to false
				ROS_INFO("Leaving Iteration Req");
			}
		}
		//Fixed Workspace for now:
		if(offset_quadposn[0] < 0.8 && offset_quadposn[0] > 0.4 && offset_quadposn[1] < 1.5 && offset_quadposn[1] > 1 && offset_quadposn[2] < 2.0 && offset_quadposn[2] > 1.5)
		{
			if(!followtraj)//If not following the gcop trajectory then set the goal position [Redundancy]
			{
				goalcount = 40;//TODO Figure out as a function of freq of goaltimer
				//Set goalyaw also to face towards the object:
				//[DEBUG]	cout<<"Goal Yaw: "<<atan2(OBJ_QUAD_origin_inoptitrackframe[1], OBJ_QUAD_origin_inoptitrackframe[0])<<endl; //atan2(y,x)
				goalyaw = atan2(OBJ_QUAD_origin_inoptitrackframe[1], OBJ_QUAD_origin_inoptitrackframe[0]); 
				//cout<<"Goal Yaw: "<<goalyaw<<endl;
				diff_goal.setValue((-curr_goal[0] + offset_quadposn[0])/goalcount, (-curr_goal[1] + offset_quadposn[1])/goalcount,(-curr_goal[2] + offset_quadposn[2])/goalcount);//Adding offset in y posn assuming that is the dirxn of approach later have to use that from the object pose
				newcamdata = true;//Specifying arm that new camera data arrived
			}
			
		}
		else
		{
			cout<<"Object out of workspace: "<<offset_quadposn[0]<<"\t"<<offset_quadposn[1]<<"\t"<<offset_quadposn[2]<<endl;
			cout<<"Object in Quad frame: \t"<<OBJ_QUAD_origin[0]<<"\t"<<OBJ_QUAD_origin[1]<<"\t"<<OBJ_QUAD_origin[2]<<endl;
		}
	}
	/*else//Full Camera Control not yet correctly implemented so commented out
	{
		controllers::ctrl_command rescmd;
		tf::Vector3 imurpy ;
		vector3MsgToTF(data.rpydata , imurpy);
		ctrlrinst->CamSet(OBJ_QUAD_stamptransform, imurpy, rescmd);
		if(!parserinstance)
		{
			ROS_WARN("Parser not instantiated");
			return;
		}
		rescmdmsg.x = rescmd.roll; rescmdmsg.y = rescmd.pitch; rescmdmsg.z = rescmd.rateyaw; rescmdmsg.w = rescmd.thrust;//For Display purposes
		if(testctrlr)//When testing dont throttle too high 
		{
			rescmdmsg.w = 0.2*data.thrustbias;//Thrustbias comes from Parser
		}
		if(data.armed && startcontrol)
		{
			if((++ratecount) == throttlecmdrate)//Throttling down the rate of sending commands to the uav
			{
				ratecount = 0;
				parserinstance->cmdrpythrust(rescmdmsg,true);//Also controlling yaw	
				//ROS_INFO("Setting cmd");
			}
		}	
	}
	*/
	if(enable_logging)
	{
		geometry_msgs::TransformStamped objmsg;
		transformStampedTFToMsg(OBJ_QUAD_stamptransform,objmsg);//converts to the right format 

		//Logging save to file
		camfile<<(OBJ_QUAD_stamptransform.stamp_.toNSec())<<"\t"<<(objmsg.transform.translation.x)<<"\t"<<(objmsg.transform.translation.y)<<"\t"<<(objmsg.transform.translation.z)<<"\t"<<(objmsg.transform.rotation.x)<<"\t"<<(objmsg.transform.rotation.y)<<"\t"<<(objmsg.transform.rotation.z)<<"\t"<<(objmsg.transform.rotation.w)<<"\t"<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<"\t"<<object_origin[0]<<"\t"<<object_origin[1]<<"\t"<<object_origin[2]<<"\t"<<(UV_O.stamp_.toNSec())<<endl;
	}
}



void QuadcopterGui::vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)//Removed arm stuff from this
{
	//Check if vrpn is skipping:
	/*if((currframe->header.stamp - UV_O.stamp_).toSec() > 0.1)
	{
		ROS_WARN("No Vrpn data from callback for greater than 0.1 sec");
	}
	*/
	transformStampedMsgToTF(*currframe,UV_O);//converts to the right format  and stores the message
	ctrlrinst->Update(UV_O);//Updates the internal state

	Matrix3x3 rotmat = UV_O.getBasis();
	rotmat.getEulerYPR(vrpnrpy[2],vrpnrpy[1],vrpnrpy[0]);
	if(enable_logging)
	{
		//Logging save to file
		vrpnfile<<(UV_O.stamp_.toNSec())<<"\t"<<(currframe->transform.translation.x)<<"\t"<<(currframe->transform.translation.y)<<"\t"<<(currframe->transform.translation.z)<<"\t"<<(currframe->transform.rotation.x)<<"\t"<<(currframe->transform.rotation.y)<<"\t"<<(currframe->transform.rotation.z)<<"\t"<<(currframe->transform.rotation.w)<<"\t"<<bias_vrpn[0]<<"\t"<<bias_vrpn[1]<<"\t"<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<endl;
	}	
	//[DEBUG] if(!startcontrol)
#ifndef LOG_DEBUG
	if(!data.armed)//Once the quadcopter is armed we do not set the goal position to quad's origin, the user will set the goal. But the goal will not move until u set the enable_control The user should not give random goal once it is initialized.
	{
		curr_goal = UV_O.getOrigin();//set the current goal to be same as the quadcopter origin we dont care abt the orientation as of now
		ctrlrinst->setgoal(curr_goal[0],curr_goal[1],curr_goal[2],goalyaw);//Set the goal to be same as the current position of the quadcopter the velgoal is by default 0
	}
#endif

	//Set the altitude of the quadcopter in the data
	parserinstance->setaltitude(currframe->transform.translation.z);
	//#ifdef PRINT
	//ROS_INFO("Rescmd: %f\t%f\t%f",rescmdmsg.x,rescmdmsg.y,rescmdmsg.w);
	//#endif
}

void QuadcopterGui::cmdtimerCallback(const ros::TimerEvent &event)
{
	/////Add a clause for doing this when camcallback is not running///////
	//Call the ctrlr to set the ctrl and then send the command to the quadparser
	if(!ctrlrinst)
	{
		ROS_WARN("Controller not instantiated");
		return;
	}
	//Check how delayed cmdtimer is :
	//cout<<(event.current_real - event.last_real).toSec()<<endl;
	if((event.current_real - event.last_real).toSec() > 0.04)
	{
		ROS_WARN("CmdTimer not catching up");
		ROS_INFO("Current Expected: %f\t%f",event.current_expected.toSec(), event.current_real.toSec());
	}

	controllers::ctrl_command rescmd;

	//vrpnrpy = vrpnrpy - bias_vrpn;//Adjusting for the bias here itself
	//errorrpy = (vrpnrpy - bias_vrpn) - tf::Vector3(data.rpydata.x, data.rpydata.y,data.rpydata.z);// No Need to do this since we are resetting imu to vrpn every 10 Hz
	//errorrpy = -bias_vrpn;//Since error is substracted from the cmd and bias should be substracted from the cmd value
	//- is used for previous method remove it when using new method

	//tf::Vector3 &quadorigin = UV_O.getOrigin();

	if(enable_camctrl && !cam_partialcontrol)//This implies we are doing full camera control
	{
		//Not Using motion capture But still need UV_O in partial cam ctrl so always useful to save the currframe when available if not available thats fine
		return;
	}

	//Store the current position of the quadcopter for display
	//Using kalman filter
	ctrlrinst->Getctrl(rescmd);//Since imu is corrected using unbiased vrpn data we can send commands which are unbiased too 

	//ctrlrinst->Set(UV_O,errorrpy, rescmd);//By default no filtering if needed can add filter data
	//cout<<"Rescmd: "<<rescmd.roll <<"\t"<<rescmd.pitch <<"\t"<<rescmd.rateyaw <<"\t"<<rescmd.thrust <<"\t"<<endl;
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
			ratecount = 0;
			parserinstance->cmdrpythrust(rescmdmsg,true);//Also controlling yaw	
			//ROS_INFO("Setting cmd");
	}	
}

void QuadcopterGui::paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level)
{
	//ROS_INFO("Request recvd");
	// Use the config values to set the goals and gains for quadcopter
//	ros::Time debugtime = ros::Time::now();
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
		ros::param::get("/ctrlr/traj_amp",config.traj_amp);
		ros::param::get("/ctrlr/traj_freq",config.traj_freq);
		config.roll_bias = bias_vrpn[0]*(180.0/M_PI);
		config.pitch_bias = bias_vrpn[1]*(180.0/M_PI);
		reconfiginit = true;
	}
	//ROS_INFO("Time taken for reconfiginit: %f",(ros::Time::now() - debugtime).toSec());
	ctrlrinst->setgains(config.kpr, config.kdr, config.kpt, config.kdt, config.kit);//Need to add kpy and kiy TODO
	ctrlrinst->setbounds(config.throtbound, (M_PI/180.0)*config.rpbound);//This throttle bound is different from throttle bias which needs to be estimated for some quadcopters. This just cuts off the throttle values that are beyond thrustbias +/- throtbound

	throttlecmdrate = config.cmdrate_throttle;

	//ROS_INFO("Time taken for 2: %f",(ros::Time::now() - debugtime).toSec());

	//setting perturbation parameters
	/*traj_freq = config.traj_freq;
	traj_amp = config.traj_amp;
	traj_skew = config.traj_skew;
	*/
	if(level&0x0002)
	{
		tf::Vector3 quadorigin = UV_O.getOrigin();
		if(updategoal_dynreconfig)//Read from the current goal if this flag is set
		{
			config.xg = curr_goal[0];
			config.yg = curr_goal[1];
			config.zg = curr_goal[2];
			config.yawg = goalyaw;
			updategoal_dynreconfig = false;
			config.update_goal = false;
		}
		else
		{
			if(config.update_goal)
			{
				goalyaw = config.yawg;
			}
			//Should not put anything else in level 2 #IMPORTANT
			//[DEBUG]if(startcontrol)
			if(data.armed)
			{
				if(config.update_goal)
				{
					goalcount = config.goalT*(5);//Just a hack to ensure the total time is ok TODO
					diff_goal.setValue((-curr_goal[0] + config.xg)/goalcount, (-curr_goal[1] + config.yg)/goalcount,(-curr_goal[2] + config.zg)/goalcount);
					//goaltimer.start();
				}
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
			if(config.update_goal)
			{
				config.update_goal = false;
			}
		}
	}
	//ROS_INFO("Time taken for 3: %f",(ros::Time::now() - debugtime).toSec());
	//publish a trajectory marker
	/*
	for(int count1 = 0;count1 <= 30;count1++)
	{
		trajectoryPtr->points[count1].x = config.xg + traj_amp*cos((count1/30.0)*2*M_PI);
		trajectoryPtr->points[count1].y = config.yg;
		trajectoryPtr->points[count1].z = config.zg + traj_amp*traj_skew*sin((count1/30.0)*2*M_PI);
	}
	targetPtr->pose.position.x = target[0];
	targetPtr->pose.position.y = target[1];
	targetPtr->pose.position.z = target[2];
	*/
	//desiredtraj_pub.publish(trajectoryPtr);
	//desiredtraj_pub.publish(targetPtr);
	
#ifdef ARM_ENABLED
	if((level&0x0008))
	{
		if(parserinstance)
		{
			parserinstance->grip(config.gripper_state);
			timer_relaxgrip.setPeriod(ros::Duration(1));//2 seconds
			timer_relaxgrip.start();//Start oneshot timer;
		}
		if(arm_hardwareinst)
		{
			arm_hardwareinst->powermotors(config.power_motors);
		}
	}
#endif

	//bias_vrpn[0] = config.roll_bias*(M_PI/180.0);
	//bias_vrpn[1] = config.pitch_bias*(M_PI/180.0);

	//traj_axis = config.traj_axis;
	//corrected_thrustbias = config.add_thrustbias + data.thrustbias;
	//ctrlrinst->setextbias(corrected_thrustbias);//Set the corrected thrustbias
//	ROS_INFO("Time taken for Param req: %f",(ros::Time::now() - debugtime).toSec());
}

//Not much load to run this timer
void QuadcopterGui::goaltimerCallback(const ros::TimerEvent &event)
{

	//Check how delayed cmdtimer is :
	if((event.current_real - event.last_real).toSec() > 0.06)
	{
		ROS_WARN("GoalTimer not catching up");
		ROS_INFO("Current Expected: %f\t%f",event.current_expected.toSec(), event.current_real.toSec());
	}

	double armres = -1e3;//Initialize arm result
#ifdef ARM_ENABLED
	if(armratecount++ == armcmdrate)//Add one to armrate count
		armratecount = 1; 
	//Also Need to set the frequency with which this is done [TODO]
	//if(enable_camctrl)//Only compute and log arm angles when camera is enabled
	if(arminst && arm_hardwareinst)//Check 
	{
		//Find the current arm angles
		arm_hardwareinst->getcurrentstate((double*)actual_armstate);//Gets both arm angle and vel
		actual_armstate[0] = parsernode::common::map_angle(actual_armstate[0]);
		//cout<<"Actual Arm Angles_ 0: "<<actual_armstate[0]<<"\t"<<actual_armstate[1]<<endl;
		//Map to the right frame i.e when all angles are 0, the arm is facing perpendicular and down with x axis also being down
		actual_armstate[0] = actual_armstate[0] < (M_PI/2)? actual_armstate[0]+M_PI/2:actual_armstate[0] - 1.5*M_PI;
		//[DEBUG]
		//cout<<"Actual Arm Angles: "<<actual_armstate[0]<<"\t"<<actual_armstate[1]<<endl;
		arminst->Fk(tip_position, actual_armstate, false);

		
		//Also publishing the current joint states to see the actual quadcopter and arm model in rviz:
/*		if(armratecount == armcmdrate)
		{
			jointstate_msg.header.stamp = ros::Time::now();
			jointstate_msg.position[0] = actual_armstate[0];
			jointstate_msg.position[1] = actual_armstate[1];
			jointstate_pub.publish(jointstate_msg);
		}
		*/
	}

  //Find the object location in local quad frame
	tf::Vector3 target_location;
	if(enable_camctrl)
		 target_location = (OBJ_QUAD_stamptransform.getOrigin() + quatRotate(UV_O.getRotation().inverse(),object_armoffset)) - arm_basewrtquad;

	if(armratecount == armcmdrate)
	{
		if(enable_logging && enable_camctrl)
		{
			//Logging save to file
			tipfile<<(ros::Time::now().toNSec())<<"\t"<<	tip_position[0]<<"\t"<<tip_position[1]<<"\t"<<tip_position[2]<<"\t"<<target_location[0]<<"\t"<<target_location[1]<<"\t"<<target_location[2]<<"\t"<<actual_armstate[0]<<"\t"<<actual_armstate[1]<<"\t"<<actual_armstate[2]<<"\t"<<actual_armstate[3]<<"\t"<<cmd_armstate[0]<<"\t"<<cmd_armstate[1]<<"\t"<<cmd_armstate[2]<<"\t"<<cmd_armstate[3]<<"\t"<<endl;//Later will change this to include timestamp when the serial data is got in a parallel thread TODO
		}
	}
#endif

	
	if(!followtraj)
	{
		if(goalcount > 0)
		{
			curr_goal = curr_goal + diff_goal;
			if(!ctrlrinst)
			{
				ROS_WARN("Controller not instantiated");
				return;
			}
			ctrlrinst->setgoal(curr_goal[0],curr_goal[1],curr_goal[2],goalyaw);//By default the vel = 0;
			goalcount--;

			//Publishing State of Quadcopter
			tf::Transform goal_frame; //Publishing goal ourselves instead of SetptCtrl
			goal_frame.setIdentity();
			goal_frame.setOrigin(curr_goal);
			broadcaster->sendTransform(tf::StampedTransform(goal_frame,ros::Time::now(),UV_O.frame_id_,"goal_posn"));
		}

		if(!enable_joy && enable_camctrl && newcamdata)//only control arm using target etc when camera is enabled and joystick is disabled
		{
			if(( armratecount == armcmdrate))//default makes it 50/4 = 12.5Hz
			{
#ifdef ARM_ENABLED
				if(arminst && parserinstance && arm_hardwareinst) //Can also add startcontrol flag for starting this only when controller has started TODO
				{
					//Verify the value of armres when we are like 5 cm from the goal posn. We will use that to calibrate the arm to open
					//Once it is opened, we note the time and put a timeout of 5 sec and then disable the cam to get a different trajectory
					//Computing the commanding arm angles
					//tf::Vector3 target_location = (OBJ_QUAD_stamptransform.getOrigin() + quatRotate(UV_O.getRotation().inverse(),object_armoffset)) - arm_basewrtquad;//Find the object location in local quad frame
					armlocaltarget[0] = target_location[0]; armlocaltarget[1] = target_location[1]; armlocaltarget[2] = target_location[2];
					//This offset is done in local frame which does not make sense always have to see what this amounts to
					armres = arminst->Ik(as,armlocaltarget);
					int solnindex = 1;//When localtargetz < 0 //For now only choosing lower elbow Later can specify which one to pick TODO
					//Convert the angles into right frame i.e when all as zero that means the arm is perpendicular and facing down
					cmd_armstate[0] = as[solnindex][1]>(-M_PI/2)?as[solnindex][1]-M_PI/2:as[solnindex][1]+1.5*M_PI;
					cmd_armstate[1] = as[solnindex][2];//Relative angle wrt to first joint no transformation needed
					//cout<<"Resulting arm angles"<<as[solnindex][0]<<"\t"<<cmd_armstate[0]<<"\t"<<cmd_armstate[1]<<endl;

					/*if(enable_logging)//Put another logging statement for Followtraj 
					{
						//Logging save to file
						tipfile<<(ros::Time::now().toNSec())<<"\t"<<	tip_position[0]<<"\t"<<tip_position[1]<<"\t"<<tip_position[2]<<"\t"<<target_location[0]<<"\t"<<target_location[1]<<"\t"<<target_location[2]<<"\t"<<actual_armstate[0]<<"\t"<<actual_armstate[1]<<"\t"<<actual_armstate[2]<<"\t"<<actual_armstate[3]<<"\t"<<cmd_armstate[0]<<"\t"<<cmd_armstate[1]<<"\t"<<cmd_armstate[2]<<"\t"<<cmd_armstate[3]<<"\t"<<endl;//Later will change this to include timestamp when the serial data is got in a parallel thread TODO
					}
					*/
					newcamdata = false;//Reset new camdata until new valid camera data arrives

					cout<<"Armres: "<<armres<<endl;
					cout<<"Arm angles: "<<cmd_armstate[0]<<"\t"<<cmd_armstate[1]<<endl;//[DEBUG]
					if(armres > -0.1) //Check if in reachable workspace
					{
						ros::Duration time_since_grabbing = ros::Time::now() - start_grabbing;
						if(time_since_grabbing.toSec() > timeout_grabbing)
						{
							//Disable_Camera and fold arm:
#ifdef LOG_DEBUG
							arm_hardwareinst->foldarm();//Can replace this with oneshot timer if needed TODO
#endif
							ui_.camcheckbox->setCheckState(Qt::Unchecked);
						}
						else
						{
							//Set the goal yaw of the quadcopter goalyaw
							//goalyaw = as[solnindex][0];//Target yaw for Quadcopter This is already done above dont need to do it here
							//Check if the tip is on the object:
							//double abssum_error = abs(tip_position[0] - armlocaltarget[0]) + abs(tip_position[1] - armlocaltarget[1]) + abs(tip_position[2] - armlocaltarget[2]);
							//[DEBUG]
							cout<<"Error in tip position: EX ["<<(tip_position[0] - armlocaltarget[0])<<"] EY ["<<(tip_position[1] - armlocaltarget[1])<<"] EZ ["<<(tip_position[2] - armlocaltarget[2])<<"]"<<endl;
							cout<<"tip position: TX ["<<(tip_position[0])<<"] TY ["<<(tip_position[1])<<"] TZ ["<<(tip_position[2])<<"]"<<endl;
							cout<<"Local Target: LX ["<<(armlocaltarget[0])<<"] LY ["<<( armlocaltarget[1])<<"] LZ ["<<(armlocaltarget[2])<<"]"<<endl;
							if( (abs(tip_position[0] - armlocaltarget[0])< 0.03) && (abs(tip_position[1] - armlocaltarget[1]) < 0.05) && (abs(tip_position[2] - armlocaltarget[2]) < 0.02) && (!gripped_already))// we will calibrate it better later //Add these as params TODO
							{
								gripped_already = true;//Stops gripping after once
//#ifndef LOG_DEBUG
								parserinstance->grip(1);//Parser does not control arm directly anymore it only controls gripper
								//Adding oneshot timer to relax grip
								timer_grabbing.setPeriod(ros::Duration(2));//5 seconds
								timer_grabbing.start();//Start oneshot timer;
//#endif
								return;
							}
							arm_hardwareinst->setarmangles(cmd_armstate);//Only using the angles with speed being constant
						}
					}
					else
					{
						start_grabbing = ros::Time::now();
					}
				}
#endif
			}
		}
	}
	else
	{
		//If following trajectory, we have to give the goal based on the trajectory information and current time
		//Need a matchnearest function for finding the closest time based on current time and provide control/goalstate information based on that
		//We avoid changing curr_goal while following trajectory, so that when we disable follow traj, the goal is set to currgoal automatically
		if(!waitingfortrajectory && !gripped_already)
		{
			//Follow the trajectory i.e set goal for quadcopter, set goals for arm based on current time and when the trajectory was req
			ros::Duration time_offset = ros::Time::now() - request_time;
			cout<<"Time Offset: "<<time_offset<<endl;//[DEBUG]
			//Find matching nearest time in the trajectory:
			int temp_index_gcoptime = nearest_index_gcoptime;//This is the original index stored to check if there is any new change
			for(int count_timesearch = nearest_index_gcoptime; count_timesearch <= gcop_trajectory.N ; count_timesearch++)
			{
				double tdiff = (gcop_trajectory.time[count_timesearch] - time_offset.toSec());
				if( tdiff > 0)
				{
					assert(count_timesearch != 0);//Since timeoffset is greater than zero and initial time in gcop ts is zero, this should not ideally happen
			/*		if((time_offset.toSec() - gcop_trajectory.time[count_timesearch - 1]) <= (-tdiff))//Checking the distance to find the nearest index
						nearest_index_gcoptime = count_timesearch - 1;
					else
						nearest_index_gcoptime = count_timesearch;
						*/
					nearest_index_gcoptime = count_timesearch;//[NOT USING NEAREST TIME BUT GIVING FUTURE CLOSEST TIME]
					break;
				}
			}
			if(nearest_index_gcoptime - temp_index_gcoptime > 0)//i.e we move from one nearest state to next nearest state, we command the quadcopter; This way even if we move to the end of trajectory, we dont provide any new goals
			{
				//nearest index gcoptime should not be zero here:
				assert(nearest_index_gcoptime > 0);
				ROS_INFO("Moving to next state: %d", nearest_index_gcoptime);
				gcop_comm::State &goalstate = gcop_trajectory.statemsg[nearest_index_gcoptime];
				if(nearest_index_gcoptime == gcop_trajectory.N)//We set the final state velocities to zero
				{
					goalstate.basetwist.linear.x = 0;
					goalstate.basetwist.linear.y = 0;
					goalstate.basetwist.linear.z = 0;
					goalstate.basetwist.angular.x = 0;
					goalstate.basetwist.angular.y = 0;
					goalstate.basetwist.angular.z = 0;
					goalstate.statevector[0] = itrq.xf.statevector[0];
					goalstate.statevector[1] = itrq.xf.statevector[1];
			///////////////		goalstate.basepose.translation.y = 1.3;//Just a hack for now 
			//////////////    goalstate.basepose.translation.z = 1.87;//Just a hack for now 
				}
				//[DEBUG]
				cout<<"Publishing goal state: "<<goalstate.basepose.translation.x<<"\t"<<goalstate.basepose.translation.y<<"\t"<<goalstate.basepose.translation.z<<"\t"<<goalyaw<<endl;
				tf::vector3MsgToTF(goalstate.basepose.translation,curr_goal);//Velocity of the goal is not currently stored
				 
				ctrlrinst->setgoal(goalstate.basepose, goalstate.basetwist); 

#ifdef ARM_ENABLED
				//ctrlrinst->setgoal(goalstate.basepose.translation.x, goalstate.basepose.translation.y, goalstate.basepose.translation.z,goalyaw);//Default velocity is 0
				//Convert the angles into right frame i.e when all as zero that means the arm is perpendicular and facing down
				if(!enable_joy)
				{
					cout<<"Publishing arm state: "<<goalstate.statevector[0]<<"\t"<<goalstate.statevector[1]<<"\t"<<goalstate.statevector[2]<<"\t"<<goalstate.statevector[3]<<endl;
					cmd_armstate[0] = goalstate.statevector[0]>(-M_PI/2)?goalstate.statevector[0]-M_PI/2:goalstate.statevector[0]+1.5*M_PI;
					cmd_armstate[1] = goalstate.statevector[1];//Relative angle wrt to first joint no transformation needed
					cmd_armstate[2] = gcop_trajectory.statemsg[nearest_index_gcoptime-1].statevector[2];//Velocities for arm should be from the previous state
					cmd_armstate[3] = gcop_trajectory.statemsg[nearest_index_gcoptime-1].statevector[3];
					arm_hardwareinst->setarmstate(cmd_armstate);//Only using the angles with speed being constant Trial 1 In trial 2 we will try with setting velocities also
				}
#endif

				//Publishing State of Quadcopter
				tf::Transform goal_frame; //Publishing goal ourselves instead of SetptCtrl
				goal_frame.setIdentity();
				goal_frame.setOrigin(curr_goal);
				broadcaster->sendTransform(tf::StampedTransform(goal_frame,ros::Time::now(),UV_O.frame_id_,"goal_posn"));

				//Also set the grabbing start time to new posn as soon as we command:
				start_grabbing = ros::Time::now();
			}
			//[DEBUG] cout<<"Nearest Index, Time offset: "<<nearest_index_gcoptime<<"\t"<<time_offset<<endl;


			///////////////Logic for Gripper Control and timeout
			ros::Duration time_since_grabbing = ros::Time::now() - start_grabbing;
			if(time_since_grabbing.toSec() > timeout_grabbing)
			{
				//Disable_Camera and fold arm:
				cout<<"Timeout Done"<<endl;
				//arm_hardwareinst->foldarm();//Can replace this with oneshot timer if needed TODO
				ui_.camcheckbox->setCheckState(Qt::Unchecked);
			}
			else
			{
#ifdef ARM_ENABLED
				//tf::Vector3 target_location = (OBJ_QUAD_stamptransform.getOrigin() + quatRotate(UV_O.getRotation().inverse(),object_armoffset)) - arm_basewrtquad;//Find the object location in local quad frame
				armlocaltarget[0] = target_location[0]; armlocaltarget[1] = target_location[1]; armlocaltarget[2] = target_location[2];
				////////Grabbing Target Code //////////////////
				cout<<"Error in tip position: EX ["<<(tip_position[0] - armlocaltarget[0])<<"] EY ["<<(tip_position[1] - armlocaltarget[1])<<"] EZ ["<<(tip_position[2] - armlocaltarget[2])<<"]"<<endl;
				cout<<"tip position: TX ["<<(tip_position[0])<<"] TY ["<<(tip_position[1])<<"] TZ ["<<(tip_position[2])<<"]"<<endl;
				cout<<"Local Target: LX ["<<(armlocaltarget[0])<<"] LY ["<<( armlocaltarget[1])<<"] LZ ["<<(armlocaltarget[2])<<"]"<<endl;
				if( (abs(tip_position[0] - armlocaltarget[0])< 0.03) && (abs(tip_position[1] - armlocaltarget[1]) < 0.05) && (abs(tip_position[2] - armlocaltarget[2]) < 0.04) && (!gripped_already))// we will calibrate it better later //Add these as params TODO
				{
					gripped_already = true;//Stops gripping after once
//#ifndef LOG_DEBUG
					parserinstance->grip(1);//Parser does not control arm directly anymore it only controls gripper
					//Adding oneshot timer to relax grip
					timer_grabbing.setPeriod(ros::Duration(2));//5 seconds
					timer_grabbing.start();//Start oneshot timer;
//#endif
					return;
				}
#endif
				//////////End Grabbing Code ///////////////////


			/*	if(enable_logging)//Put another logging statement for Followtraj TODO
				{
					//Logging save to file
					tipfile<<(ros::Time::now().toNSec())<<"\t"<<	tip_position[0]<<"\t"<<tip_position[1]<<"\t"<<tip_position[2]<<"\t"<<target_location[0]<<"\t"<<target_location[1]<<"\t"<<target_location[2]<<"\t"<<actual_armstate[0]<<"\t"<<actual_armstate[1]<<"\t"<<actual_armstate[2]<<"\t"<<actual_armstate[3]<<"\t"<<cmd_armstate[0]<<"\t"<<cmd_armstate[1]<<"\t"<<cmd_armstate[2]<<"\t"<<cmd_armstate[3]<<"\t"<<endl;//Later will change this to include timestamp when the serial data is got in a parallel thread TODO
				}
				*/
			}
		}
	}
}

}
PLUGINLIB_DECLARE_CLASS(rqt_quadcoptergui, QuadcopterGui, rqt_quadcoptergui::QuadcopterGui, rqt_gui_cpp::Plugin)
