#include <rqt_quadcoptergui/onboardnodehandler.h>
#define ARM_ENABLED
//#define ARM_MOCK_TEST_DEBUG

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh_):nh(nh_)
                                                            , broadcaster(new tf::TransformBroadcaster())
                                                            , logdir_created(false), enable_logging(false)
                                                            , publish_rpy(false)
                                                            , enable_tracking(false), enable_velcontrol(false), enable_rpytcontrol(false)
                                                            , reconfig_init(false), reconfig_update(false)
                                                            , last_roi_update_time_(0)
                                                            , desired_yaw_rate(0), feedforward_yaw(0)
                                                            //, armcmdrate(4), armratecount(0), gripped_already(false), newcamdata(false)
                                                            //, enable_control(false), enable_integrator(false), enable_camctrl(false), enable_manualtargetretrieval(false)
                                                            //, tip_position(), goalcount(1), diff_goal(), count_imu(0)
{
  //Load Parameters:
  ROS_INFO("Loading Parameters");
  loadParameters();

  //initialize member variables
  ROS_INFO("Setting up Member Variables");
  setupMemberVariables();
 
  ROS_INFO("Creating Parser");
  if(!createParserInstance())
  {
    ROS_ERROR("Failed to create Quadcopter Parser");
    return;
  }

/*#ifdef ARM_ENABLED
  ROS_INFO("Creating Arm Hardware Instance");
  arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
#endif */

  ROS_INFO("Subscribing to Callbacks");
  //Subscribe to GuiCommands
  gui_command_subscriber_ = nh_.subscribe("/gui_commands", 10, &OnboardNodeHandler::receiveGuiCommands, this);
  //Subscribe to roi
  roi_subscriber_ = nh_.subscribe("roi", 10, &OnboardNodeHandler::receiveRoi, this);
  camera_info_subscriber_ = nh.subscribe("camera_info",1,&OnboardNodeHandler::receiveCameraInfo, this);
  //Subscribe to Camera Estimator:
  //camdata_sub = nh_.subscribe("/Pose_Est/objpose",1,&OnboardNodeHandler::camcmdCallback,this);

  ROS_INFO("Advertising Topics");
  //Advertise Gui State:
  gui_state_publisher_ = nh_.advertise<rqt_quadcoptergui::GuiStateMessage>("/gui_state", 10);
  //Advertise Quad Parser Data:
  quad_state_publisher_ = nh_.advertise<std_msgs::String>("/quad_status", 10);
  //Advertise Joint States of Manipulator
  //jointstate_pub = nh_.advertise<sensor_msgs::JointState>("/movingrobot/joint_states",10);
  //Advertise Target velocity for tracking
  vel_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("targetvel", 10);

  if(publish_rpy)
  {
    //advertise rpy topics:
    imu_rpy_pub_ = nh_.advertise<geometry_msgs::Vector3>("imu_rpy",10);
  }

  //Connect to dynamic reconfigure server:
	ROS_INFO("Setting Up Reconfigure Sever");
  reconfigserver.reset(new dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>(nh_));
  reconfigcallbacktype = boost::bind(&OnboardNodeHandler::paramreqCallback, this, _1, _2);
  reconfigserver->setCallback(reconfigcallbacktype);

  //Create Timers:
  ROS_INFO("Creating Timers");
  //Timer to move goal dynamically
  //goaltimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::goaltimerCallback,this);//50Hz So the goal can go upto 25 Hz  Nyquist rate
  //goaltimer.stop();
  //Timer for grabbing object
  //timer_grabbing = nh_.createTimer(ros::Duration(4), &OnboardNodeHandler::closeAfterGrabbing, this, true);//One shot timer
  //timer_grabbing.stop();
  //Timer for relaxing gripper
  //timer_relaxgrip = nh_.createTimer(ros::Duration(4), &OnboardNodeHandler::oneshotGrab, this, true);//One shot timer
  //timer_relaxgrip.stop();
  //Timer for commanding quadcopter
  cmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::cmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  cmdtimer.stop();
  rpytimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::rpytimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  rpytimer.stop();
  //Timer for sending quadcopter state
  quadstatetimer = nh_.createTimer(ros::Duration(0.05), &OnboardNodeHandler::quadstatetimerCallback, this);//20Hz update GUI quad state
}

OnboardNodeHandler::~OnboardNodeHandler()
{
  //Poweroff arm:
/*#ifdef ARM_ENABLED
  arm_hardwareinst->powermotors(false);
#endif
*/
  parserinstance.reset();
  parser_loader.reset();
  //ctrlrinst.reset();
  //arminst.reset();
/*#ifdef ARM_ENABLED
  arm_hardwareinst.reset();
#endif
*/
  //vrpndata_sub.shutdown();
  //camdata_sub.shutdown();
  //joydata_sub.shutdown();
  //gcoptraj_sub.shutdown();
  //gui_command_subscriber_.shutdown();

  gui_state_publisher_.shutdown();
  quad_state_publisher_.shutdown();
  //jointstate_pub.shutdown();
  //armtarget_pub.shutdown();
  //iterationreq_pub.shutdown();

  broadcaster.reset();

  reconfigserver.reset();

  //goaltimer.stop();
  cmdtimer.stop();
  quadstatetimer.stop();

  //vrpnfile.close();//Close the file
  camfile.close();//Close the file
  tipfile.close();//Close the file
}

//////////////////////HELPER Functions///////////////////
void OnboardNodeHandler::publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg)
{
    gui_state_publisher_.publish(state_msg);
}

void OnboardNodeHandler::setupMemberVariables()
{
  // Prepare Target cube pointer for visualizing object
  vel_marker.id = 1;
  vel_marker.header.frame_id = uav_name;
  vel_marker.action = visualization_msgs::Marker::ADD;
  vel_marker.type = visualization_msgs::Marker::ARROW;
  //Setting points
  geometry_msgs::Point pt;
  pt.x = pt.y = pt.z = 0;
  vel_marker.points.push_back(pt);
  pt.z = 0.1;//Start
  vel_marker.points.push_back(pt);
  vel_marker.scale.x = 0.02;//Shaft dia
  vel_marker.scale.y = 0.05;//Head Dia
  vel_marker.color.r = 1.0;//red arrow
  vel_marker.color.a = 1.0;
  //Initial command:
  rpytcmd.x = rpytcmd.y = rpytcmd.z = 0;
  rpytcmd.w = 10;
}

inline void OnboardNodeHandler::loadParameters()
{
  if(!nh.getParam("/gui/parser_plugin",parserplugin_name))
  {
    parserplugin_name = "pixhawk";//Default
    ROS_ERROR("Cannot find parser_plugin parameter to load the parser");
  }

  //Tracking Parameters
  nh.param<double>("/tracking/vel_mag",vel_mag, 0.1);
  nh.param<double>("/tracking/yaw_gain",yaw_gain, 0.01);

  //Where the arm base is in quadcopter's frame Orientations are assumed to be known #TODO Add them as a paremeter too
  /*nh.param<double>("/ctrlr/armbasewrtquadx",arm_basewrtquad[0],0.0732);
  nh.param<double>("/ctrlr/armbasewrtquady",arm_basewrtquad[1], 0.0);
  nh.param<double>("/ctrlr/armbasewrtquadz",arm_basewrtquad[2], -0.07);//was -0.1
*/

  nh.param<std::string>("/gui/uav_name",uav_name,"dji");
  nh.param<std::string>("/gui/logdir",logdir,"/home/gowtham");
  nh.param<bool>("/gui/publishrpy",publish_rpy,false);

  ROS_INFO("UAV Name: %s",uav_name.c_str());

  //Find Camera pose in Quad frame
  static tf::TransformListener listener;//Will make it a class member later (for other functions to use TODO)
  try{
    bool result = listener.waitForTransform(uav_name, "camera",
                                            ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform(uav_name, "camera",
                             ros::Time(0), CAM_QUAD_transform);
    if(!result)
      cout<<"Cannot find QUAD to CAM Transform"<<endl;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

inline bool OnboardNodeHandler::createParserInstance()
{
  if(!parser_loader)
    parser_loader.reset(new pluginlib::ClassLoader<parsernode::Parser>("parsernode","parsernode::Parser"));

  try
  {
    parserinstance = parser_loader->createInstance(parserplugin_name);
    parserinstance->initialize(nh);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    return false;
  }

  //Wait till parser is initialized:
  ros::Time current_time = ros::Time::now();
  while((ros::Time::now() - current_time).toSec() < 15)
  {
    if(parserinstance->initialized)
      break;
    usleep(100000);//0.1 sec
  }
  //parserinstance->getquaddata(data);

  //Set the Gripper to Relaxed State:
  parserinstance->grip(0);

  return parserinstance->initialized;
}

inline void OnboardNodeHandler::setupLogDir()
{
  // Logger
  std::string logdir_append = logdir + "/session";
  std::string logdir_stamped = parsernode::common::addtimestring(logdir_append);
  ROS_INFO("Creating Log dir: %s",logdir_stamped.c_str());
  int status = mkdir(logdir_stamped.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);//Create the directory see http://pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
  if(status != 0)
  {
      ROS_WARN("Cannot create directory");
  }

  //vrpnfile.open((logdir_stamped+"/vrpn.dat").c_str());
	//Check if we cannot open file:
    /*if(!vrpnfile.is_open())
	{
		ROS_WARN("Cannot Open File");
		return;
	}
  vrpnfile.precision(9);
	vrpnfile.rdbuf()->pubsetbuf(vrpnfile_buffer,FILE_BUFFER_SIZE);//Set Buffer:
  vrpnfile<<"#Time\tPos.X\tPos.Y\tPos.Z\tQuat.X\tQuat.Y\tQuat.Z\tQuat.W\tBias_R\tBias_P\tCurr_goal_x\tCurr_goal_y\tCurr_goal_z"<<endl;

  //Camfile
  camfile.open((logdir_stamped+"/campose.dat").c_str());
  camfile.precision(9);
	camfile.rdbuf()->pubsetbuf(camfile_buffer,FILE_BUFFER_SIZE);//Set Buffer
  camfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z \t Quat.X \t Quat.Y \t Quat.Z \t Quat.W\t Curr_goal.x\t Curr_goal.y\t Curr_goal.z\t Obj_origin.x\t Obj_origin.y\t Obj_origin.z"<<endl;

  //Arm Tip file
  tipfile.open((logdir_stamped+"/tippos.dat").c_str());
  tipfile.precision(9);
	tipfile.rdbuf()->pubsetbuf(tipfile_buffer, FILE_BUFFER_SIZE);//Tip File Buffer
  tipfile<<"#Time \t Pos.X \t Pos.Y \t Pos.Z\t DesPos.X\t DesPos.Y\t DesPos.Z\t Act_armangle0\t Act_armangle1\t Des_armangle0\t Des_armangle1"<<endl;

    */
  if(parserinstance)
    parserinstance->setlogdir(logdir_stamped);

  logdir_created = true;//Specify that log directory has been created
}

////////////////////////STATE TRANSITIONS/////////////////////////
inline void OnboardNodeHandler::stateTransitionLogging(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Log");
		goto PUBLISH_LOGGING_STATE;
  }
  if(state == true)
  {
    if(enable_logging == false)
    {
      if(logdir_created == false)
      {
        setupLogDir();//Create a log directory and files corresponding
      }
      enable_logging = true;
      parserinstance->controllog(true);
    }
  }
  else
  {
    if(enable_logging == true)
    {
      ROS_INFO("I am called2");
      enable_logging = false;
      parserinstance->controllog(false);
    }
  }
  //Publish Change of State:
PUBLISH_LOGGING_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_logging;
  state_message.commponent_id = state_message.log_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionTracking(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Track");
		goto PUBLISH_TRACKING_STATE;
  }

  //Check if we are in air and control is enabled; otherwise do not track
  if(data.armed && enable_velcontrol)
  {
    enable_tracking = state;
    //If we are switching of tracking set desired vel to 0
    if(!enable_tracking)
    {
      desired_vel.x = desired_vel.y = desired_vel.z = desired_yaw_rate = 0;
      reconfig_update = true;
    }
  }
  
  //Publish Change of State:
PUBLISH_TRACKING_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_tracking;
  state_message.commponent_id = state_message.tracking_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionVelControl(bool state)
{
  bool result = false;

  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Control Quad");
    goto PUBLISH_VEL_CONTROL_STATE;
  }
  if(enable_rpytcontrol)
      stateTransitionRpytControl(false);

  //First check if tracking is on; If on, switch it off:
  if(enable_tracking)
  {
    ROS_INFO("Setting Tracking to False");
    stateTransitionTracking(false);
  }

  enable_velcontrol = state;
  ROS_INFO("State: %d",enable_velcontrol);

  //Stop Control Timer if state is false:
  if(!enable_velcontrol)
  {
    ROS_INFO("Stopping Cmdtimer");
    cmdtimer.stop();
  }

  if(data.armed && enable_velcontrol)//If we are in air and asked to enable control of quadcopter
  {
    ROS_INFO("Calling Enable Control");
    result = parserinstance->flowControl(enable_velcontrol);//get control
    if(!result)
    {
        enable_velcontrol = false;
        ROS_WARN("Failed to get control of quadcopter");
    }
    else
    {
      desired_vel.x = desired_vel.y = desired_vel.z = desired_yaw_rate = 0;
      reconfig_update = true;
      //Start Timer to send vel to quadcopter
      cmdtimer.start();
    }
  }
  else
  {
      ROS_INFO("Releasing Control of Quadcopter");
      enable_velcontrol = false;
      result = parserinstance->flowControl(enable_velcontrol);//get control
      if(!result)
      {
        ROS_WARN("Failed to release control of quadcopter");
      }
  }
  
  //Publish Change of State:
PUBLISH_VEL_CONTROL_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_velcontrol;
  state_message.commponent_id = state_message.vel_control_status;
  gui_state_publisher_.publish(state_message);
}

void OnboardNodeHandler::stateTransitionRpytControl(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot create rpyt control");
    goto PUBLISH_RPYT_CONTROL_STATE;
  }
  if(!parserinstance->initialized)
    goto PUBLISH_RPYT_CONTROL_STATE;
  if(enable_velcontrol)
      stateTransitionVelControl(false);

  if(state)
  {
    bool result = parserinstance->flowControl(true);//get control

    if(!result)
    {
      ROS_INFO("Cannot open sdk");
      goto PUBLISH_RPYT_CONTROL_STATE;
    }
    //Enable rpyttimer:
    ROS_INFO("Starting rpy timer");
    rpytimer.start();
    enable_rpytcontrol = true;
  }
  else
  {
    ROS_INFO("Stopping rpy timer");
    rpytimer.stop();
    enable_rpytcontrol = false;
    //Set current vel to 0:
    desired_vel.x = desired_vel.y = desired_vel.z = feedforward_yaw = 0;
    parserinstance->cmdvelguided(desired_vel, feedforward_yaw);
  }
    //Publish Change of State:
PUBLISH_RPYT_CONTROL_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_rpytcontrol;
  state_message.commponent_id = state_message.rpyt_control_status;
  gui_state_publisher_.publish(state_message);
}

////////////////////////Gui Button Commands////////////
inline void OnboardNodeHandler::armQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  if(!parserinstance->initialized)
  {
    return;
  }
  parserinstance->takeoff();
}

inline void OnboardNodeHandler::disarmQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  stateTransitionTracking(false);
  ROS_INFO("Stopping cmd timer");
  cmdtimer.stop();
  parserinstance->disarm();
  stateTransitionVelControl(false);
  stateTransitionLogging(false);
}

inline void OnboardNodeHandler::landQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  stateTransitionTracking(false);//Stop Tracking
  ROS_INFO("Stopping cmd timer");
  cmdtimer.stop();
  parserinstance->land();
  stateTransitionLogging(false);
}

////////////////////////CALLBACKS//////////////////////
void OnboardNodeHandler::receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg)
{
  //Do whatever is commanded
  switch(command_msg.commponent_name)
  {
  case command_msg.enable_log://0
    stateTransitionLogging(command_msg.command);
    break;
  case command_msg.enable_tracking://1
    stateTransitionTracking(command_msg.command);
    break;
  case command_msg.enable_vel_control://2
    stateTransitionVelControl(command_msg.command);
    break;
  case command_msg.enable_rpyt_control://2
    stateTransitionRpytControl(command_msg.command);
    break;
  case command_msg.arm_quad ://6
    ROS_INFO("Arming Quad");
    armQuad();
    break;
  case command_msg.land_quad://7
    ROS_INFO("Landing Quad");
    landQuad();
    break;
  case command_msg.disarm_quad://8
    ROS_INFO("Disarming Quad");
    disarmQuad();
    break;
  }
}

void OnboardNodeHandler::receiveCameraInfo(const sensor_msgs::CameraInfo &info)
{
  intrinsics.reset(new sensor_msgs::CameraInfo());
  *intrinsics = info;//Copy
  camera_info_subscriber_.shutdown();
}

void OnboardNodeHandler::receiveRoi(const sensor_msgs::RegionOfInterest &roi_rect)
{
  last_roi_update_time_ = ros::Time::now();
  if(!intrinsics || !parserinstance) 
  {
    ROS_WARN("No Camera Info received/ No Parser instance created");
    return;
  }
  geometry_msgs::Vector3 temp_desired_vel;
  double temp_desired_yaw_rate;
  //Get RPY:
  parserinstance->getquaddata(data);
  roiToVel(roi_rect,
           data.rpydata, *intrinsics,
           CAM_QUAD_transform,vel_mag,yaw_gain,
           temp_desired_vel, temp_desired_yaw_rate);
  if(enable_tracking)
  {
    desired_vel = temp_desired_vel;
    desired_yaw_rate = temp_desired_yaw_rate;
  }
/*  if(enable_tracking)
  {
    parserinstance->cmdvelguided(desired_vel, desired_yaw_rate);
    //Publish vector to rviz
  }
  */
  //Publish velocity
  vel_marker.points[1].x = temp_desired_vel.x; vel_marker.points[1].y = temp_desired_vel.y; vel_marker.points[1].z = temp_desired_vel.z;
  vel_marker_pub_.publish(vel_marker); 
}

void OnboardNodeHandler::paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config, uint32_t level)
{
  // Use the config values to set the goals and gains for quadcopter
  if(!parserinstance)
  {
    ROS_WARN("Parser not defined");
    return;
  }

  if(!reconfig_init)
  {
    config.yaw_gain = yaw_gain;
    config.vel_mag = vel_mag;
    reconfig_init = true;
    return;
  }
  if(reconfig_update || enable_tracking)//If we are tracking or reconfig is being updated
  {
    config.vx = desired_vel.x;
    config.vy = desired_vel.y;
    config.vz = desired_vel.z;
    config.yaw_rate = desired_yaw_rate;
    config.update_vel = false;
    reconfig_update = false;
    return;
  }


  if(level&0x0002)
  {
    if(data.armed)
    {
      if(config.update_vel)
      {
        desired_vel.x = config.vx; desired_vel.y = config.vy; desired_vel.z = config.vz; desired_yaw_rate = config.yaw_rate;
        ROS_INFO("Desired vel:  %f, %f, %f, %f",desired_vel.x, desired_vel.y, desired_vel.z, desired_yaw_rate);
      }
    }
    else
    {
      config.vx = 0;
      config.vy = 0;
      config.vz = 0;
      ROS_INFO("Quad not armed");
    }
    config.update_vel = false;
  }

  yaw_gain = config.yaw_gain;
  vel_mag = config.vel_mag;

/*#ifdef ARM_ENABLED
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
*/
}


void OnboardNodeHandler::quadstatetimerCallback(const ros::TimerEvent &event)
{
  if(!parserinstance)
  {
    //ROS_ERROR("No parser instance created");
    return;
  }

  parserinstance->getquaddata(data);

//If we have to publish rpy data:
  if(publish_rpy)
  {
    geometry_msgs::Vector3 rpymsg;
    rpymsg.x = (data.rpydata.x)*(180/M_PI);
    rpymsg.y = (data.rpydata.y)*(180/M_PI);
    rpymsg.z = (data.rpydata.z)*(180/M_PI);
    imu_rpy_pub_.publish(rpymsg);
  }
  //Convert data servo_in to rpytcmd:
  if(!enable_rpytcontrol)
  {
    rpytcmd.x = parsernode::common::map(data.servo_in[0],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.y = parsernode::common::map(data.servo_in[1],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.w = parsernode::common::map(data.servo_in[2],-10000, 10000, 10, 100);
    rpytcmd.z = parsernode::common::map(data.servo_in[3],-10000, 10000, -M_PI, M_PI);
  }
  // Create a Text message based on the data from the Parser class
  sprintf(buffer,
          "Battery Percent: %2.2f\t\nlx: %2.2f\tly: %2.2f\tlz: %2.2f\nAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\n"
          "Magx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\n"
          "Trvelx: %2.2f\tTrvelz: %2.2f\tTryawr: %2.2f\nCmdr: %2.2f\tCmdp: %2.2f\tCmdt: %2.2f\tCmdy: %2.2f\nMass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s",
          data.batterypercent
          ,data.localpos.x, data.localpos.y, data.localpos.z
          ,data.altitude
          ,data.rpydata.x*(180/M_PI),data.rpydata.y*(180/M_PI),data.rpydata.z*(180/M_PI)//IMU rpy angles
          ,data.magdata.x,data.magdata.y,data.magdata.z
          ,data.linacc.x,data.linacc.y,data.linacc.z
          ,data.linvel.x,data.linvel.y,data.linvel.z
          ,desired_vel.x,desired_vel.z,desired_yaw_rate*(180/M_PI)
          ,rpytcmd.x*(180/M_PI), rpytcmd.y*(180/M_PI), rpytcmd.w, rpytcmd.z*(180/M_PI)
          ,data.mass,data.timestamp,data.quadstate.c_str());

  //Publish State:
  std_msgs::String string_msg;
  string_msg.data = std::string(buffer);
  quad_state_publisher_.publish(string_msg); 
  //Publish TF of the quadcopter position:
  tf::Transform quad_transform(tf::createQuaternionFromYaw(data.rpydata.z), tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
  broadcaster->sendTransform(tf::StampedTransform(quad_transform, ros::Time::now(), "world", uav_name));
}

void OnboardNodeHandler::rpytimerCallback(const ros::TimerEvent& event)
{
  if(parserinstance)
  {
    parserinstance->getquaddata(data);
    rpytcmd.x = parsernode::common::map(data.servo_in[0],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.y = parsernode::common::map(data.servo_in[1],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.w = parsernode::common::map(data.servo_in[2],-10000, 10000, 10, 100);
    rpytcmd.z = parsernode::common::map(data.servo_in[3],-10000, 10000, -M_PI, M_PI);
    ROS_INFO("Timer Running");
    parserinstance->cmdrpythrust(rpytcmd, true);
  }
}

void OnboardNodeHandler::cmdtimerCallback(const ros::TimerEvent& event)
{
  //Check if roi has not been updated for more than 0.5 sec; Then disable tracking automatically:
  if(enable_tracking)
  {
    if((ros::Time::now() - last_roi_update_time_).toSec() > 0.5)
    {
      ROS_WARN("Roi has not been updated for 0.5 sec");
      stateTransitionTracking(false);
    }
  }
  //DEBUG: If tracking is enabled only see the values of vel rather than command it
  //Feedforward yaw:
  feedforward_yaw = feedforward_yaw + desired_yaw_rate*(ros::Time::now() - event.last_real).toSec();
  if(feedforward_yaw > 180)//Wrapping Around
    feedforward_yaw = feedforward_yaw - 360;
  if(feedforward_yaw < -180)
    feedforward_yaw = feedforward_yaw + 360;
  //send command of the velocity
  if(parserinstance)
    parserinstance->cmdvelguided(desired_vel, feedforward_yaw);
}

