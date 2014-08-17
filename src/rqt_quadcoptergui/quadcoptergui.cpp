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

#include <rqt_quadcoptergui/quadcoptergui.h>

#include <pluginlib/class_list_macros.h>

namespace rqt_quadcoptergui {

QuadcopterGui::QuadcopterGui() : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
	, goalcount(0),goalyaw(0)
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
	, yoffset_object(0.55), updategoal_dynreconfig(false), cam_partialcontrol(true)
	, trajectoryPtr(new visualization_msgs::Marker())
	, targetPtr(new visualization_msgs::Marker())
	, broadcaster(new tf::TransformBroadcaster())
	, timeout_grabbing(3)
	{
		setObjectName("QuadcopterGui");
		parser_loader.reset(new pluginlib::ClassLoader<parsernode::Parser>("parsernode","parsernode::Parser"));
		UV_O.setIdentity();
		errorrpy.setValue(0,0,0);
		target.setValue(0,0,0);//Initializing the target extraction point
		quadtobase.setIdentity();
		quadtobase.setOrigin(tf::Vector3(0.0732,0,-0.07));//The z distance needs to adjusted exactly
		//armpwm.resize(3);//Number of arms for now just hardcoded

		targetPtr->id = 1;
		targetPtr->ns = "targetpickup";
		targetPtr->header.frame_id = "/optitrak";
		targetPtr->action = visualization_msgs::Marker::ADD;
		targetPtr->pose.orientation.w = 1.0;
		targetPtr->type = visualization_msgs::Marker::CUBE;
		targetPtr->scale.x = 0.1;
		targetPtr->scale.y = 0.1;
		targetPtr->scale.z = 0.1;
		targetPtr->color.r = 1.0;
		targetPtr->color.a = 1.0;


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
		//For now default value of object_offset:
		object_armoffset = tf::Vector3(0,0.05,-0.07);//relative to the markers in Optitrack frame //For full camera control this SHOULD BE IN Object/Inertial Frame
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

	//stringdata_pub = nh.advertise<std_msgs::String>("stringout",1);

	//Load Target:
	nh.getParam("/ctrlr/targetx",target[0]);
	nh.getParam("/ctrlr/targety",target[1]);
	nh.getParam("/ctrlr/targetz",target[2]);
	nh.getParam("/ctrlr/partialcam_control",cam_partialcontrol);
	nh.getParam("/ctrlr/timeout_grabbing",timeout_grabbing);
	//For now fixed value later will set this as a parameter:
	arm_basewrtquad.setValue(0.0732, 0, -0.1);

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
	timer->start(50);//20Hz
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
	ctrlrinst.reset(new CameraSetptCtrl(nh, broadcaster));
	arminst.reset(new gcop::Arm);
	int dyn_deviceInd = 0;//Defaults
	int dyn_baudnum = 57600;
	nh.getParam("/dynamixel/deviceIndex",dyn_deviceInd);
	nh.getParam("/dynamixel/baudrate",dyn_baudnum);
	arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
	arminst->l1 = 0.175;
	//arminst->l2 = 0.35;
	arminst->l2 = 0.42;
	arminst->x1 = 0.025;//Need to change this after measuring again TODO
	parserinstance->getquaddata(data);
	ctrlrinst->setextbias(data.thrustbias); //Fext initial guess comes from the parser. We will need to estimate it for some quadcopters if its used in commanding it.
	//bias_vrpn.setValue(2.33*(M_PI/180),-0.3*(M_PI/180),0);
	bias_vrpn.setValue(0,0,0);
	nh.getParam("/bias_vrpnroll",bias_vrpn[0]);
	nh.getParam("/bias_vrpnpitch",bias_vrpn[1]);
	nh.getParam("/bias_vrpnyaw",bias_vrpn[2]);
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
		vrpnfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z \t Quat.X \t Quat.Y \t Quat.Z \t Quat.W"<<endl;
		//Camfile
		camfile.open((logdir_stamped+"/campose.dat").c_str());//TODO add warning if we cannot open the file
		camfile.precision(9);
		camfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z \t Quat.X \t Quat.Y \t Quat.Z \t Quat.W"<<endl;
		//Arm Tip file
		tipfile.open((logdir_stamped+"/tippos.dat").c_str());//TODO add warning if we cannot open the file
		tipfile.precision(9);
		tipfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z"<<endl;
		//cmdfile.open(logdir_stamped+"/cmd.dat");//TODO add warning if we cannot open the file
		parserinstance->setlogdir(logdir_stamped);
		ctrlrinst->setlogdir(logdir_stamped);
	}

	//subscribe to vrpndata now
	string uav_posename;
	if(!nh.getParam("/gui/vrpn_pose",uav_posename))
	{
		ROS_ERROR("Cannot load uav pose parameter");
		return;
	}
	cout<<"Subscribing to uav pose topic on: "<<uav_posename<<endl;
	vrpndata_sub = nh.subscribe(uav_posename,1,&QuadcopterGui::cmdCallback,this);
	camdata_sub = nh.subscribe("/Pose_Est/objpose",1,&QuadcopterGui::camcmdCallback,this);
	joydata_sub = nh.subscribe("/joy",1,&QuadcopterGui::joyCallback,this);

	desiredtraj_pub = nh.advertise<visualization_msgs::Marker>("desired_traj", 5);

	//Connect to dynamic reconfigure server:
	reconfigserver.reset(new dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>(nh));
	reconfigcallbacktype = boost::bind(&QuadcopterGui::paramreqCallback, this, _1, _2);
	reconfigserver->setCallback(reconfigcallbacktype);
	//Create timer for moving Goal Dynamically:
	goaltimer = nh.createTimer(ros::Duration(0.02), &QuadcopterGui::goaltimerCallback,this);//50Hz So the goal can go upto 25 Hz  Nyquist rate
	goaltimer.stop();
	timer_grabbing = nh.createTimer(ros::Duration(4), &QuadcopterGui::ClosingafterGrabbing, this, true);//One shot timer
	timer_grabbing.stop();
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
	if(ui_.bias_estcheckbox->isChecked())
	{
		bias_vrpn += (1/(bias_count+1))*(vrpnrpy - bias_vrpn);
		bias_count += 1;//Increase the count
	}
	//Get the Arm angles for Tip Position:
	if(!enable_camctrl)//If camera is enabled the angles are checked over there already
	{
		arm_hardwareinst->getcurrentangles((double*)actual_armangles);
		//cout<<"Actual Arm Angles_ -1: "<<actual_armangles[0]<<"\t"<<actual_armangles[1]<<endl;
		actual_armangles[0] = parsernode::common::map_angle(actual_armangles[0]);
		//cout<<"Actual Arm Angles_ 0: "<<actual_armangles[0]<<"\t"<<actual_armangles[1]<<endl;
		//Map to the right frame as explained in previous comments
		actual_armangles[0] = actual_armangles[0] < (M_PI/2)? actual_armangles[0]+M_PI/2:actual_armangles[0] - 1.5*M_PI;
		//[DEBUG]
		//cout<<"Actual Arm Angles: "<<actual_armangles[0]<<"\t"<<actual_armangles[1]<<endl;

		if(arminst)//Simple Check to avoid errors
			arminst->Fk(tip_position, actual_armangles, false);
	}
	//cout<<"Bias :"<<bias_vrpn[0]<<"\t"<<bias_vrpn[1]<<"\t"<<bias_vrpn[2]<<"\t"<<endl;
	//errorrpy.setValue(0,0,0);
	tf::Vector3 quadorigin = UV_O.getOrigin();
	tf::Vector3 obj_origin = OBJ_QUAD_stamptransform.getOrigin();
	// Create a Text message based on the data from the Parser class
	sprintf(buffer,
			"Battery Percent: %2.2f\t\nTemperature: %2.2f\tPressure: %2.2f\tWindspeed: %2.2f\tAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\nMagx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\nposx: %2.2f\tposy: %2.2f\tposz: %2.2f\nvrpnr: %2.2f\tvrpnp: %2.2f\tvrpny: %2.2f\nErrorr: %2.2f\tErrorrp: %2.2f\tErrory: %2.2f\nresr: %2.2f\tresp: %2.2f\tresy: %2.2f\trest: %2.2f\nbias_r: %2.2f\tbias_p: %2.2f\tbias_y: %2.2f\nObjx: %2.2f\tObjy: %2.2f\tObjz: %2.2f\t\nTipx: %2.2f\tTipy: %2.2f\tTipz: %2.2f\t\nMass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s", 
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
	ros::NodeHandle nh = getNodeHandle();
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
	}
  if(arm_hardwareinst && parserinstance)
	{
		arm_hardwareinst->foldarm();//Just fold the arm whenever you switch between two modes
		if(!enable_joy)
			parserinstance->grip(-1);//Open gripper position in automatic position so that it will grip it automatically
	}
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
			parserinstance->grip(-1);//Open We changed open to be more energy efficient by increasing pwm width TODO/Done but not uploaded
		if(cam_partialcontrol)//If partial control to debug we have to see the output of the goal whenever the quad is tarted up
		{
			goaltimer.start();//Redundancy
		}
		else
		{
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
		//Fold arm:
		//if(arm_hardwareinst)
			//arm_hardwareinst->foldarm();
		//Also specify the goal as the current quad postion TODO
		//curr_goal = UV_O.getOrigin();//set the current goal to be same as the quadcopter origin we dont care abt the orientation as of now
		tf::Vector3 centergoal(0.75, 0.9, 1.4);//Center of workspace with same height
		updategoal_dynreconfig = true;//Set the flag to make sure dynamic reconfigure reads the new goal
		goalcount = 20; //Set the goal back to the specified posn smoothly
		diff_goal.setValue((-curr_goal[0] + centergoal[0])/goalcount, (-curr_goal[1] + centergoal[1])/goalcount,(-curr_goal[2] + centergoal[2])/goalcount);
		diff_velgoal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
		goaltimer.start();//Redundancy
	}
}
/*
void QuadcopterGui::Capture_Target()//Removed as it is not useful anymore once camera is there
{
	if(!ctrlrinst || !parserinstance || !arminst)
	{
		ROS_WARN("Cannot find ctrlr or parser or arm");
		return;
	}
	// We want to capture the position of the final target from knowing the transformation of the quadcopter, known joint angles
	//and  the transformation from quad to base of the arm using forward kinematics
	buttoncount1 = (buttoncount1+1)%2;
	//The button should be pressed two times for first time it will open the arm and wait for you to press the second time when it will capture it
	if(buttoncount1 == 1)
	{
		armangles[0] = 0;
		armangles[1] = 0;
		armangles[2] = 0;//Neutral This is not an angle
		if(arm_hardwareinst)
			arm_hardwareinst->setarmangles(armangles);//Set the angles
	}
	else if(buttoncount1 == 0)
	{
		//Matrix3x3 rotmat = UV_O.getBasis();
		//tf::Vector3 vrpnrpy;
		//rotmat.getEulerYPR(vrpnrpy[2],vrpnrpy[1],vrpnrpy[0]);
		armangles[0] = 0; armangles[1] = 0; armangles[2] = 0;//Set the  current arm angles
		//The yaw of the quadcopter is directly used no need to set armgoal[0] 
		double terminal_pos[3];
		arminst->Fk(terminal_pos,armangles);
		tf::Vector3 localpos(terminal_pos[0],terminal_pos[1],terminal_pos[2]);
		target = UV_O*(quadtobase*localpos);//Setting the target with respect to global frame
		ROS_INFO("Target Position: %f\t%f\t%f",terminal_pos[0],terminal_pos[1],terminal_pos[2]);

		//Done setting the target folding back the arm
		if(arm_hardwareinst)
			arm_hardwareinst->foldarm();
	}
}
*/

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
		ctrlrinst->integratethrust(false);//Redundancy 
		return;
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

void QuadcopterGui::follow_trajectory(int state)
{
	if(state == Qt::Checked)
	{
		followtraj = true;
	}
	else if(state == Qt::Unchecked)
	{
		followtraj = false;
		goalcount = 1;
		diff_goal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
		diff_velgoal.setValue(0,0,0);//Will set the goal back to the curr_goal without perturbations
		//////Stop the arm movement TODO Also add a checkbox for the arm and uncheck it when not following the trajectory
	}
}

void QuadcopterGui::shutdownPlugin()
{
	//Poweroff arm:
	arm_hardwareinst->powermotors(false);
	parserinstance.reset();
	parser_loader.reset();
	ctrlrinst.reset();
	arminst.reset();
	arm_hardwareinst.reset();
	vrpndata_sub.shutdown();
	reconfigserver.reset();
	goaltimer.stop();
	vrpnfile.close();//Close the file
	camfile.close();//Close the file
	tipfile.close();//Close the file
	trajectoryPtr.reset();
	targetPtr.reset();
	desiredtraj_pub.shutdown();
	//cmdfile.close();//Close the file
	//stringdata_sub.shutdown();
	//stringdata_pub.shutdown();
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
		parserinstance->grip(1);//Hold
	else if(buttoncount == 2)
		parserinstance->grip(-1);//Release
	//For starting to move arm:

	if(enable_joy)//If the joystick is not enable return
	{
		if((joymsg->buttons[1] - joymsg_prevbutton1) > 0)
		{
			buttoncount1 = (buttoncount1+1)%2;
			cout<<buttoncount1<<endl;
		}
		joymsg_prevbutton1 = joymsg->buttons[1];
		if(buttoncount1 == 1)
		{
			armpwm[0] = joymsg->axes[1];
			armpwm[1] = joymsg->axes[0];//Will convert them also into angles later
			if(arm_hardwareinst)
				arm_hardwareinst->setarmpwm(armpwm);
			//cout<<"Settting armpwm"<<endl;
		}
		else
		{
			if(arm_hardwareinst)
				arm_hardwareinst->foldarm();
		}
	}
}

void QuadcopterGui::ClosingafterGrabbing(const ros::TimerEvent &event)
{
	ROS_INFO("Closing the arm and grabbing target");
	if(arm_hardwareinst && parserinstance)
	{
								arm_hardwareinst->foldarm();
								parserinstance->grip(0);//Neutral
	}
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
	OBJ_QUAD_stamptransform  = tf::StampedTransform(OBJ_QUAD_transform, currframe->header.stamp,uav_name,"object");//CAM_QUAD is the pose of Camera in Quadcopter frame, OBJ_MOD is the transform needed to make the object parallel (in terms of roll, pitch) with the Inertial frame
	if(!enable_camctrl)//Not Using Camera Control
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
	// Find the current tip Position (this is common whether we use full or partial cam control)
	arm_hardwareinst->getcurrentangles((double*)actual_armangles);
	actual_armangles[0] = parsernode::common::map_angle(actual_armangles[0]);
	//cout<<"Actual Arm Angles_ 0: "<<actual_armangles[0]<<"\t"<<actual_armangles[1]<<endl;
	//Map to the right frame i.e when all angles are 0, the arm is facing perpendicular and down with x axis also being down
	actual_armangles[0] = actual_armangles[0] < (M_PI/2)? actual_armangles[0]+M_PI/2:actual_armangles[0] - 1.5*M_PI;
	//[DEBUG]
	//cout<<"Actual Arm Angles: "<<actual_armangles[0]<<"\t"<<actual_armangles[1]<<endl;
	arminst->Fk(tip_position, actual_armangles, false);
	if(enable_logging)
	{
		tipfile<<(ros::Time::now().toNSec())<<"\t"<<	tip_position[0]<<"\t"<<tip_position[1]<<"\t"<<tip_position[2]<<"\t"<<endl;//Later will change this to include timestamp when the serial data is got in a parallel thread TODO
	}
	//If using partialcontrol should not directly control quadcopter instead just set the goal in optitrack frame and use the optitrack controller to do the job:
	if(cam_partialcontrol)
	{
		//Set the goal based on OBJ_QUAD transform 
		tf::Transform OBJ_OPTITRACK_transform = UV_O*OBJ_QUAD_transform;
		tf::Vector3 object_origin = OBJ_OPTITRACK_transform.getOrigin();
		tf::Vector3 OBJ_QUAD_origin = OBJ_QUAD_transform.getOrigin();
		tf::Vector3 OBJ_QUAD_origin_inoptitrackframe = quatRotate(UV_O.getRotation(),OBJ_QUAD_origin);//Get the object in quadcopters frame  written in optirack frame
		//Substract y offset (Assuming the object is to be approached in y dirxn) TODO Use object pose or some input dirxn of approach for grasping later//
		///object_origin[1] -= yoffset_object;
		double object_offsetposny = object_origin[1] - yoffset_object;
		//Fixed Workspace for now:
		if(object_origin[0] < 1.4 && object_origin[0] > 0.1 && object_offsetposny < 1.6 && object_offsetposny > 0.2 && object_origin[2] < 1.6 && object_origin[2] > 0)
		{
			goalcount = 20;//TODO Figure out as a function of freq of goaltimer
			//Set goalyaw also to face towards the object:
			//[DEBUG]	cout<<"Goal Yaw: "<<atan2(OBJ_QUAD_origin_inoptitrackframe[1], OBJ_QUAD_origin_inoptitrackframe[0])<<endl; //atan2(y,x)
			goalyaw = atan2(OBJ_QUAD_origin_inoptitrackframe[1], OBJ_QUAD_origin_inoptitrackframe[0]); 
			//cout<<"Goal Yaw: "<<goalyaw<<endl;
			diff_goal.setValue((-curr_goal[0] + object_origin[0])/goalcount, (-curr_goal[1] + object_offsetposny)/goalcount,(-curr_goal[2] + object_origin[2])/goalcount);//Adding offset in y posn assuming that is the dirxn of approach later have to use that from the object pose
			diff_velgoal.setValue(0,0,0);
			//cout<<"Goal posn in optitrack: "<<object_origin[0]<<"\t"<<object_origin[1]<<"\t"<<object_origin[2]<<endl;

			/*************** ARM CODE *********************/
			//Computing the arm angles
			//double yawdiff = atan2(OBJ_QUAD_origin[1], OBJ_QUAD_origin[0]);  Will try to improve the offset later
			tf::Vector3 target_location = (OBJ_QUAD_origin + quatRotate(UV_O.getRotation().inverse(),object_armoffset)) - arm_basewrtquad;//Find the object location in local quad frame
			armlocaltarget[0] = target_location[0]; armlocaltarget[1] = target_location[1]; armlocaltarget[2] = target_location[2];
			//This offset is done in local frame which does not make sense always have to see what this amounts to
			double armres = arminst->Ik(as,armlocaltarget);
			int solnindex = 1;//When localtargetz < 0 //For now only choosing lower elbow Later can specify which one to pick TODO
			//Convert the angles into right frame i.e when all as is all zero that means the arm is perpendicular and facing down
			armangles[0] = as[solnindex][1]>(-M_PI/2)?as[solnindex][1]-M_PI/2:as[solnindex][1]+1.5*M_PI;
			//armangles[1] = as[solnindex][2]>(-M_PI/2)?as[solnindex][2]-M_PI/2:as[solnindex][2]+1.5*M_PI;
			armangles[1] = as[solnindex][2];//Relative angle wrt to first joint no transformation needed
			armangles[2] = armpwm[2];//Using the joystick for gripping  
			//cout<<"Resulting arm angles"<<as[solnindex][0]<<"\t"<<armangles[0]<<"\t"<<armangles[1]<<endl;

			if(!enable_joy)
			{
				if((++armratecount == armcmdrate))//default makes it 60/4 = 15Hz
				{
					armratecount = 0;
					if(arminst && parserinstance && arm_hardwareinst) //Can also add startcontrol flag for starting this only when controller has started TODO
					{
						//Verify the value of armres when we are like 5 cm from the goal posn. We will use that to calibrate the arm to open
						//Once it is opened, we note the time and put a timeout of 5 sec and then disable the cam to get a different trajectory
						/*if(OBJ_QUAD_origin.length2() <  (yoffset_object+0.1)*(yoffset_object))//The distance object to quad is less than 0.5 m yoffset
						{
							ROS_INFO("Obj_quad_in_optframe: [%f]\t[%f] \tArmres: [%f]",OBJ_QUAD_origin_inoptitrackframe[0], OBJ_QUAD_origin_inoptitrackframe[1], armres);
						}
						*/
						cout<<"Armres: "<<armres<<endl;
						cout<<"Arm angles: "<<armangles[0]<<"\t"<<armangles[1]<<endl;
						if(armres > -0.1) //Check if in reachable workspace
						{
							ros::Duration time_since_grabbing = ros::Time::now() - start_grabbing;
							if(time_since_grabbing.toSec() > timeout_grabbing)
							{
								//Disable_Camera and fold arm:
								ui_.camcheckbox->setCheckState(Qt::Unchecked);
								//parserinstance->foldarm();
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
									if( (abs(tip_position[0] - armlocaltarget[0])< 0.05) && (abs(tip_position[1] - armlocaltarget[1]) < 0.05) && (abs(tip_position[2] - armlocaltarget[2]) < 0.02) )// we will calibrate it better later //Add these as params TODO
									{
										parserinstance->grip(1);//Parser does not control arm directly anymore it only controls gripper
										//Add oneshot timer to relax grip
										timer_grabbing.setPeriod(ros::Duration(5));//5 seconds
										timer_grabbing.start();//Start oneshot timer;
										ui_.camcheckbox->setCheckState(Qt::Unchecked);
										return;
									}
									arm_hardwareinst->setarmangles(armangles);
							}
						}
						else
						{
								start_grabbing = ros::Time::now();
						}
					}
				}
			}
		}
		else
		{
			cout<<"Object out of workspace: "<<object_origin[0]<<"\t"<<object_offsetposny<<"\t"<<object_origin[2]<<endl;
			cout<<"Object in Quad frame: \t"<<OBJ_QUAD_origin[0]<<"\t"<<OBJ_QUAD_origin[1]<<"\t"<<OBJ_QUAD_origin[2]<<endl;
		}
	}
	else//Full Camera Control
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
	if(enable_logging)
	{
		geometry_msgs::TransformStamped objmsg;
		transformStampedTFToMsg(OBJ_QUAD_stamptransform,objmsg);//converts to the right format 
		//Logging save to file
		camfile<<(OBJ_QUAD_stamptransform.stamp_.toNSec())<<"\t"<<(objmsg.transform.translation.x)<<"\t"<<(objmsg.transform.translation.y)<<"\t"<<(objmsg.transform.translation.z)<<"\t"<<(objmsg.transform.rotation.x)<<"\t"<<(objmsg.transform.rotation.y)<<"\t"<<(objmsg.transform.rotation.z)<<"\t"<<(objmsg.transform.rotation.w)<<endl;
	}
}
void QuadcopterGui::cmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)//Removed arm stuff from this
{
	/////Add a clause for doing this when camcallback is not running///////
	//Call the ctrlr to set the ctrl and then send the command to the quadparser
	if(!ctrlrinst)
	{
		ROS_WARN("Controller not instantiated");
		return;
	}
	controllers::ctrl_command rescmd;
	transformStampedMsgToTF(*currframe,UV_O);//converts to the right format 
	Matrix3x3 rotmat = UV_O.getBasis();
	rotmat.getEulerYPR(vrpnrpy[2],vrpnrpy[1],vrpnrpy[0]);
	errorrpy = (vrpnrpy - bias_vrpn) - tf::Vector3(data.rpydata.x, data.rpydata.y,data.rpydata.z);
	if(enable_camctrl && !cam_partialcontrol)//This implies we are doing full camera control
	{
		//Not Using motion capture But still need UV_O in partial cam ctrl so always useful to save the currframe when available if not available thats fine
		return;
	}
	//Store the current position of the quadcopter for display
	//ctrlrinst->Set(UV_O, rescmd);
	ctrlrinst->Set(UV_O, errorrpy, rescmd);//By default no filtering if needed can add filter data
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
		if((++ratecount) == throttlecmdrate)//Throttling down the rate of sending commands to the uav
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
	//[DEBUG] if(!startcontrol)
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
		ros::param::get("/ctrlr/traj_amp",config.traj_amp);
		ros::param::get("/ctrlr/traj_freq",config.traj_freq);
		reconfiginit = true;
	}
	ctrlrinst->setgains(config.kpr, config.kdr, config.kpt, config.kdt, config.kit);
	ctrlrinst->setbounds(config.throtbound, (M_PI/180.0)*config.rpbound);//This throttle bound is different from throttle bias which needs to be estimated for some quadcopters. This just cuts off the throttle values that are beyond thrustbias +/- throtbound

	throttlecmdrate = config.cmdrate_throttle;

	//setting perturbation parameters
	traj_freq = config.traj_freq;
	traj_amp = config.traj_amp;
	traj_skew = config.traj_skew;
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
		}
		else
		{
			goalyaw = config.yawg;
			//Should not put anything else in level 2 #IMPORTANT
			//[DEBUG]if(startcontrol)
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
	}
	//publish a trajectory marker
	for(int count1 = 0;count1 <= 30;count1++)
	{
		trajectoryPtr->points[count1].x = config.xg + traj_amp*cos((count1/30.0)*2*M_PI);
		trajectoryPtr->points[count1].y = config.yg;
		trajectoryPtr->points[count1].z = config.zg + traj_amp*traj_skew*sin((count1/30.0)*2*M_PI);
	}
	targetPtr->pose.position.x = target[0];
	targetPtr->pose.position.y = target[1];
	targetPtr->pose.position.z = target[2];
	desiredtraj_pub.publish(trajectoryPtr);
	desiredtraj_pub.publish(targetPtr);
	if(parserinstance)
	{
		parserinstance->grip(config.gripper_state);
	}
	if(arm_hardwareinst)
	{
		arm_hardwareinst->powermotors(config.power_motors);
	}
	//traj_axis = config.traj_axis;
	//corrected_thrustbias = config.add_thrustbias + data.thrustbias;
	//ctrlrinst->setextbias(corrected_thrustbias);//Set the corrected thrustbias
}

//Not much load to run this timer
void QuadcopterGui::goaltimerCallback(const ros::TimerEvent &event)
{
	//if(data.quadstate == "Flying")
	//{
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
		}
		trajtime_offset = ros::Time::now();
		if(cam_partialcontrol && enable_camctrl)
		{
			//[DEBUG]cout<<"Current Goal: "<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<endl;
			//Since setptctrl is not called we should publish the goal ourselves:
			tf::Transform goal_frame;
			goal_frame.setIdentity();
			goal_frame.setOrigin(curr_goal);
			broadcaster->sendTransform(tf::StampedTransform(goal_frame,ros::Time::now(),UV_O.frame_id_,"goal_posn"));
		}
	}
	else
	{ 
		ros::Duration currduration = ros::Time::now() - trajtime_offset;
		float anglearg = 2*M_PI*traj_freq*currduration.toSec();
		//	float signal = 0, signalder = 0;

		//cout<<"Signal: "<<signal<<"\t Signalder: "<<signalder<<endl;
		diff_goal.setValue(traj_amp*cos(anglearg),0,traj_amp*traj_skew*sin(anglearg));//Can add axis for the trajectory too
		diff_velgoal.setValue(-traj_amp*2*M_PI*traj_freq*sin(anglearg),0,traj_skew*traj_amp*2*M_PI*traj_freq*cos(anglearg));//Can make this ellipse/hyperbola by adding two more parameters a, b
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
