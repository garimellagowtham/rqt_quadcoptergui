#include <rqt_quadcoptergui/onboardnodehandler.h>
#define ARM_ENABLED
//#define ARM_MOCK_TEST_DEBUG

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh_):nh(nh_)
                                                            , broadcaster(new tf::TransformBroadcaster())
                                                            , logdir_created(false), enable_logging(false)
                                                            , publish_rpy(false)
                                                            , enable_tracking(false), enable_velcontrol(false), enable_rpytcontrol(false), enable_poscontrol(false)
                                                            , enable_mpccontrol(false), enable_trajectory_tracking(false)
                                                            , reconfig_init(false), reconfig_update(false)
                                                            , desired_yaw(0), kp_trajectory_tracking(1.0), timeout_trajectory_tracking(1.0), timeout_mpc_control(1.0)
                                                            , traj_visualizer_(nh_,"world",false), model_control(nh_), mpc_closed_loop_(false), mpc_trajectory_count(0)
                                                            //, waypoint_vel(0.1), waypoint_yawvel(0.01)
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

  //Create RoiVelController:
  roi_vel_ctrlr_.reset(new RoiVelController(nh,uav_name));
  roi_vel_ctrlr_->setCameraTransform(CAM_QUAD_transform);

/*#ifdef ARM_ENABLED
  ROS_INFO("Creating Arm Hardware Instance");
  arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
#endif */

  ROS_INFO("Subscribing to Callbacks");
  //Subscribe to GuiCommands
  gui_command_subscriber_ = nh.subscribe("/gui_commands", 10, &OnboardNodeHandler::receiveGuiCommands, this);
  goal_pose_subscriber_ = nh.subscribe("goal",1,&OnboardNodeHandler::receiveGoalPose,this);
  trajectory_subscriber_ = nh.subscribe("ctrltraj",1,&OnboardNodeHandler::gcoptrajectoryCallback,this);
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
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("targetvel", 10);

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
  velcmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::velcmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  velcmdtimer.stop();
  poscmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::poscmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  poscmdtimer.stop();
  mpctimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::mpctimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  mpctimer.stop();
  rpytimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::rpytimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  rpytimer.stop();
  trajectorytimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::trajectorytimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  trajectorytimer.stop();
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
  velcmdtimer.stop();
  poscmdtimer.stop();
  mpctimer.stop();
  quadstatetimer.stop();

  //vrpnfile.close();//Close the file
  //camfile.close();//Close the file
  //tipfile.close();//Close the file
}

//////////////////////HELPER Functions///////////////////
void OnboardNodeHandler::publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg)
{
    gui_state_publisher_.publish(state_msg);
}

inline void copyPtToVec(geometry_msgs::Vector3 in, geometry_msgs::Point out)
{
    out.x = in.x; out.y = in.y; out.z = in.z;
}

void OnboardNodeHandler::setupMemberVariables()
{
  //Initial command:
  rpytcmd.x = rpytcmd.y = rpytcmd.z = 0;
  rpytcmd.w = 10;
  mpc_goalpose.position.x = 0; mpc_goalpose.position.y = 0; mpc_goalpose.position.z = 0;
  mpc_goalpose.orientation.x = 0; mpc_goalpose.orientation.y = 0; mpc_goalpose.orientation.z = 0; mpc_goalpose.orientation.w = 1;
}

inline void OnboardNodeHandler::loadParameters()
{
  if(!nh.getParam("/gui/parser_plugin",parserplugin_name))
  {
    parserplugin_name = "pixhawk";//Default
    ROS_ERROR("Cannot find parser_plugin parameter to load the parser");
  }

  //Where the arm base is in quadcopter's frame Orientations are assumed to be known #TODO Add them as a paremeter too
  /*nh.param<double>("/ctrlr/armbasewrtquadx",arm_basewrtquad[0],0.0732);
  nh.param<double>("/ctrlr/armbasewrtquady",arm_basewrtquad[1], 0.0);
  nh.param<double>("/ctrlr/armbasewrtquadz",arm_basewrtquad[2], -0.07);//was -0.1
*/

  nh.param<std::string>("/gui/uav_name",uav_name,"dji");
  nh.param<std::string>("/gui/logdir",logdir,"/home/gowtham");
  nh.param<bool>("/gui/publishrpy",publish_rpy,false);
  nh.param<double>("/mpc/goal_tolerance", goal_tolerance,0.2);//Stop when 0.2m away from goal

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

  roi_vel_ctrlr_->setlogdir(logdir_stamped);

  logdir_created = true;//Specify that log directory has been created
}

inline void OnboardNodeHandler::setInitialStateMPC()
{
  parserinstance->getquaddata(data);//Get Latest data
  model_control.setInitialState(data.localpos, data.linvel, data.linacc,
                                data.rpydata, data.omega, rpytcmd);
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
      roi_vel_ctrlr_->controllog(true);
    }
  }
  else
  {
    if(enable_logging == true)
    {
      ROS_INFO("I am called2");
      enable_logging = false;
      parserinstance->controllog(false);
      roi_vel_ctrlr_->controllog(false);
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

  if(!state)
      enable_tracking = false;

  //Check if we are in air and control is enabled; otherwise do not track
  if(data.armed && enable_velcontrol)
  {
    enable_tracking = state;
    //If we are switching of tracking set desired vel to 0
    if(!enable_tracking)
    {
      desired_vel.x = desired_vel.y = desired_vel.z = 0;
      reconfig_update = true;
    }
    else//Tracking is Being Enabled
    {
      parserinstance->getquaddata(data);
      geometry_msgs::Vector3 des_obj_dir;
      bool result = roi_vel_ctrlr_->setDesiredObjectDir(data.rpydata,des_obj_dir);
      if(!result)
      {
          ROS_WARN("Failed to initialize des obj dir");
          enable_tracking = false;
      }
      else
      {
        //Get Desired Obj direction and Publish the marker
        visualization_msgs::Marker dirxn_marker;
        dirxn_marker.id = 2;
        dirxn_marker.header.frame_id = "world";
        dirxn_marker.action = visualization_msgs::Marker::ADD;
        dirxn_marker.type = visualization_msgs::Marker::ARROW;

        geometry_msgs::Point pt;
        copyPtToVec(data.localpos,pt);
        dirxn_marker.points.push_back(pt);

        pt.x += 5.0*des_obj_dir.x;
        pt.y += 5.0*des_obj_dir.y;
        pt.z += 5.0*des_obj_dir.z;
        dirxn_marker.points.push_back(pt);
        dirxn_marker.scale.x = 0.02;//Shaft dia
        dirxn_marker.scale.y = 0.05;//Head Dia
        dirxn_marker.color.b = 1.0;//Blue arrow
        dirxn_marker.color.a = 1.0;
        marker_pub_.publish(dirxn_marker);
      }
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
  // Check if other states are on:
  if(enable_rpytcontrol)
      stateTransitionRpytControl(false);
  if(enable_tracking)
    stateTransitionTracking(false);
  if(enable_poscontrol)
    stateTransitionPosControl(false);
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);
  if(enable_trajectory_tracking)
    stateTransitionTrajectoryTracking(false);

  enable_velcontrol = state;
  ROS_INFO("State: %d",enable_velcontrol);

  //Stop Control Timer if state is false:
  if(!enable_velcontrol)
  {
    ROS_INFO("Stopping Cmdtimer");
    velcmdtimer.stop();
  }

  if(data.armed && enable_velcontrol)//If we are in air and asked to enable control of quadcopter
  {
    ROS_INFO("Calling Enable Control");

    result = parserinstance->flowControl(true);//get control
    if(!result)
    {
      enable_velcontrol = false;
      ROS_WARN("Failed to get control of quadcopter");
    }
    else
    {
      desired_vel.x = desired_vel.y = desired_vel.z = 0;
      desired_yaw = data.rpydata.z;
      reconfig_update = true;
      //Start Timer to send vel to quadcopter
      velcmdtimer.start();
    }
  }
  
  
  //Publish Change of State:
PUBLISH_VEL_CONTROL_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_velcontrol;
  state_message.commponent_id = state_message.vel_control_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionPosControl(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Control Quad");
    goto PUBLISH_POS_CONTROL_STATE;
  }

  // Check if other states are on:
  if(enable_rpytcontrol)
      stateTransitionRpytControl(false);
  if(enable_tracking)
    stateTransitionTracking(false);
  if(enable_velcontrol)
    stateTransitionVelControl(false);
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);
  if(enable_trajectory_tracking)
    stateTransitionTrajectoryTracking(false);

  enable_poscontrol = state;
  ROS_INFO("State: %d",enable_poscontrol);

  if(!enable_poscontrol)
  {
      poscmdtimer.stop();
  }
  else
  {
      ROS_INFO("Calling Enable Control");
      bool result = parserinstance->flowControl(true);//get control
      if(result)
      {
        parserinstance->getquaddata(data);
        goal_position = data.localpos;
        goal_altitude = goal_position.z;
        reconfig_update = true;
        desired_yaw = data.rpydata.z;
        poscmdtimer.start();
      }
      else
      {
        ROS_WARN("Failed to Obtain Control of Quadcopter");
      }
  }

  //Publish Change of State:
PUBLISH_POS_CONTROL_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_poscontrol;
  state_message.commponent_id = state_message.pos_control_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionMPCControl(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Control Quad");
    goto PUBLISH_MPC_CONTROL_STATE;
  }

  // Check if other states are on:
  if(enable_rpytcontrol)
      stateTransitionRpytControl(false);
  if(enable_tracking)
    stateTransitionTracking(false);
  if(enable_velcontrol)
    stateTransitionVelControl(false);
  if(enable_poscontrol)
    stateTransitionPosControl(false);
  if(enable_trajectory_tracking)
    stateTransitionTrajectoryTracking(false);

  enable_mpccontrol = state;
  ROS_INFO("State: %d",enable_mpccontrol);

  if(!enable_mpccontrol)
  {
      mpctimer.stop();
      //Set current vel to 0:
      desired_vel.x = desired_vel.y = desired_vel.z = 0;
      desired_yaw = data.rpydata.z;
      parserinstance->cmdvelguided(desired_vel, desired_yaw);
      //Set current trajectory count to 0:
      mpc_trajectory_count = 0;
  }
  else
  {
      ROS_INFO("Calling Enable MPC Control");
      bool result = parserinstance->flowControl(true);//get control
      if(result)
      {
        parserinstance->setmode("rpyt_angle");
        ROS_INFO("Starting Timer");
        mpc_request_time = ros::Time::now();
        mpctimer.start();
      }
      else
      {
        enable_mpccontrol = false;
        ROS_WARN("Failed to Obtain Control of Quadcopter");
      }
  }

  //Publish Change of State:
PUBLISH_MPC_CONTROL_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_mpccontrol;
  state_message.commponent_id = state_message.mpc_control_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionTrajectoryTracking(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot create rpyt control");
    goto PUBLISH_TRAJECTORY_TRACKING_STATE;
  }
  if(enable_tracking)
    stateTransitionTracking(false);
  if(enable_velcontrol)
    stateTransitionVelControl(false);
  if(enable_poscontrol)
    stateTransitionPosControl(false);
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);
  if(enable_trajectory_tracking)
    stateTransitionTrajectoryTracking(false);

  if(state)
  {
    bool result = parserinstance->flowControl(true);//get control

    if(!result)
    {
      ROS_INFO("Cannot open sdk");
      goto PUBLISH_TRAJECTORY_TRACKING_STATE;
    }
    enable_trajectory_tracking = true;
  }
  else
  {
    enable_trajectory_tracking = false;
    //Stop Trajectory tracking timer:
    trajectorytimer.stop();
    //Set current vel to 0:
    desired_vel.x = desired_vel.y = desired_vel.z = 0;
    desired_yaw = data.rpydata.z;
    parserinstance->cmdvelguided(desired_vel, desired_yaw);
  }
  //Publish Change of State:
PUBLISH_TRAJECTORY_TRACKING_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_trajectory_tracking;
  state_message.commponent_id = state_message.trajectory_tracking_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionRpytControl(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot create rpyt control");
    goto PUBLISH_RPYT_CONTROL_STATE;
  }
  if(!parserinstance->initialized)
    goto PUBLISH_RPYT_CONTROL_STATE;
  if(enable_tracking)
      stateTransitionTracking(false);
  if(enable_velcontrol)
      stateTransitionVelControl(false);
  if(enable_poscontrol)
      stateTransitionPosControl(false);
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);
  if(enable_trajectory_tracking)
    stateTransitionTrajectoryTracking(false);

  if(data.armed)
  {
    if(state)
    {
      bool result = parserinstance->flowControl(true);//get control

      if(!result)
      {
        ROS_INFO("Cannot open sdk");
        goto PUBLISH_RPYT_CONTROL_STATE;
      }
      //Enable rpyttimer:
      //parserinstance->setmode("rpyt_rate");//Using Yaw rate instead of angle
      parserinstance->setmode("rpyt_angle");//Using Yaw angle with feedforward
      ROS_INFO("Starting rpy timer");
      rpytimer_start_time = ros::Time::now();
      rpytimer.start();
      enable_rpytcontrol = true;
    }
    else
    {
      ROS_INFO("Stopping rpy timer");
      rpytimer.stop();
      enable_rpytcontrol = false;
      //Set current vel to 0:
      desired_vel.x = desired_vel.y = desired_vel.z = 0;
      desired_yaw = data.rpydata.z;
      parserinstance->cmdvelguided(desired_vel, desired_yaw);
    }
  }
  else
  {
    enable_rpytcontrol = false;//Quad is not armed so no rpytcontrol should be allowed
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
  velcmdtimer.stop();
  poscmdtimer.stop();
  parserinstance->disarm();
  stateTransitionVelControl(false);
  stateTransitionPosControl(false);
  stateTransitionLogging(false);
}

inline void OnboardNodeHandler::initializeMPC()
{
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);
  //Set Goal for  MPC
  mpc_goalpose.position.z = goal_altitude;//TODO convert goal altitude into a local offset
  model_control.setGoal(mpc_goalpose);
  setInitialStateMPC();
  model_control.iterate(100);
  model_control.getCtrlTrajectory(gcop_trajectory);
  traj_visualizer_.publishTrajectory(gcop_trajectory);
}

inline void OnboardNodeHandler::landQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  stateTransitionTracking(false);//Stop Tracking
  stateTransitionVelControl(false);//Stop Vel Control
  ROS_INFO("Stopping cmd timer");
  velcmdtimer.stop();
  poscmdtimer.stop();
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
    stateTransitionLogging(true);
    stateTransitionTracking(command_msg.command);
    break;
  case command_msg.enable_vel_control://2
    stateTransitionVelControl(command_msg.command);
    break;
  case command_msg.enable_rpyt_control://2
    stateTransitionRpytControl(command_msg.command);
    break;
  case command_msg.enable_pos_control:
    stateTransitionPosControl(command_msg.command);
    break;
  case command_msg.enable_mpc_control:
    stateTransitionMPCControl(command_msg.command);
    break;
  case command_msg.enable_trajectory_tracking:
    stateTransitionTrajectoryTracking(command_msg.command);
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
  case command_msg.initialize_mpc://9
    ROS_INFO("Initializing MPC Trajectory");
    initializeMPC();
    break;
  }
}

void OnboardNodeHandler::receiveGoalPose(const geometry_msgs::PoseStamped &goal_pose)
{
  if(enable_poscontrol)
  {
    goal_position.x = goal_pose.pose.position.x;
    goal_position.y = goal_pose.pose.position.y;
    desired_yaw = tf::getYaw(goal_pose.pose.orientation);
  } 
  mpc_goalpose = goal_pose.pose;///< Only recording the pose here. It is set when u initialize MPC !!
  //TODO: Add a marker to rviz at the goal position
}

void OnboardNodeHandler::gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg)
{
    if(!enable_trajectory_tracking)//If follow trajectory has not been enabled do not receive a trajectory
      return;
    //[DEBUG]
    ROS_INFO("Received Gcop Trajectory");
    gcop_trajectory = traj_msg;//Default Copy constructor (Instead if needed can write our own copy constructor)
    //Set the last state's velocity and angular velocity to 0
    gcop_comm::State &goalstate = gcop_trajectory.statemsg[gcop_trajectory.N];
    goalstate.basetwist.linear.x = 0;
    goalstate.basetwist.linear.y = 0;
    goalstate.basetwist.linear.z = 0;
    goalstate.basetwist.angular.x = 0;
    goalstate.basetwist.angular.y = 0;
    goalstate.basetwist.angular.z = 0;
    
    nearest_index_gcop_trajectory = 0;//Every time we get a new trajectory we reset the time index for searching nearest current time to zero
    gcop_trajectory_request_time = ros::Time::now();//Record when we received request
    traj_visualizer_.publishTrajectory(gcop_trajectory);
    //start timer for tracking trajectory
    trajectorytimer.start();
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
    //Tracking Parameters
    nh.param<double>("/tracking/radial_gain",config.radial_gain);
    nh.param<double>("/tracking/tangential_gain",config.tangential_gain);
    nh.param<double>("/tracking/desired_object_distance",config.desired_object_distance);
    reconfig_init = true;
    return;
  }
  if(reconfig_update || enable_tracking)//If we are tracking or reconfig is being updated
  {
    config.vx = desired_vel.x;
    config.vy = desired_vel.y;
    config.vz = desired_vel.z;
    config.yaw = desired_yaw;
    config.goal_altitude = goal_altitude;
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
        desired_vel.x = config.vx; desired_vel.y = config.vy; desired_vel.z = config.vz; desired_yaw = config.yaw;
        ROS_INFO("Desired vel:  %f, %f, %f, %f",desired_vel.x, desired_vel.y, desired_vel.z, desired_yaw);
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
  roi_vel_ctrlr_->setGains(config.radial_gain, config.tangential_gain, config.desired_object_distance);
  goal_altitude = config.goal_altitude;
  Nit = config.Nit;
  mpc_closed_loop_ = config.mpc_closed_loop;
  kp_trajectory_tracking = config.kp_trajectory_tracking;
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
  if(!enable_rpytcontrol && !enable_mpccontrol)
  {
    rpytcmd.x = parsernode::common::map(data.servo_in[0],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.y = parsernode::common::map(data.servo_in[1],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.w = parsernode::common::map(data.servo_in[2],-10000, 10000, 10, 100);
    rpytcmd.z = data.rpydata.z;//Current yaw
    //rpytcmd.z = parsernode::common::map(data.servo_in[3],-10000, 10000, -M_PI, M_PI);
  }
  double object_distance = roi_vel_ctrlr_->getObjectDistance();
  // Create a Text message based on the data from the Parser class
  sprintf(buffer,
          "Battery Percent: %2.2f\t\nlx: %2.2f\tly: %2.2f\tlz: %2.2f\nAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\n"
          "Magx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\n"
          "Trvelx: %2.2f\tTrvely: %2.2f\tTrvelz: %2.2f\tTrObjD: %2.2f\nCmdr: %2.2f\tCmdp: %2.2f\tCmdt: %2.2f\tCmdy: %2.2f\n"
          "Goalx: %2.2f\tGoaly: %2.2f\tGoalz: %2.2f\tGoaly: %2.2f\n"
          "Mass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s",
          data.batterypercent
          ,data.localpos.x, data.localpos.y, data.localpos.z
          ,data.altitude
          ,data.rpydata.x*(180/M_PI),data.rpydata.y*(180/M_PI),data.rpydata.z*(180/M_PI)//IMU rpy angles
          ,data.magdata.x,data.magdata.y,data.magdata.z
          ,data.linacc.x,data.linacc.y,data.linacc.z
          ,data.linvel.x,data.linvel.y,data.linvel.z
          ,desired_vel.x,desired_vel.y,desired_vel.z,object_distance
          ,rpytcmd.x*(180/M_PI), rpytcmd.y*(180/M_PI), rpytcmd.w, rpytcmd.z*(180/M_PI)
          ,goal_position.x, goal_position.y, goal_position.z, desired_yaw
          ,data.mass,data.timestamp,data.quadstate.c_str());

  //Publish State:
  std_msgs::String string_msg;
  string_msg.data = std::string(buffer);
  quad_state_publisher_.publish(string_msg); 
  //Publish TF of the quadcopter position:
  //tf::Transform quad_transform(tf::createQuaternionFromYaw(data.rpydata.z), tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
  tf::Transform quad_transform(tf::createQuaternionFromRPY(data.rpydata.x, data.rpydata.y, data.rpydata.z), tf::Vector3(data.localpos.x, data.localpos.y, data.localpos.z));
  broadcaster->sendTransform(tf::StampedTransform(quad_transform, ros::Time::now(), "world", uav_name));
}

void OnboardNodeHandler::rpytimerCallback(const ros::TimerEvent& event)
{
  if(parserinstance)
  {
    parserinstance->getquaddata(data);

    if((rpytimer_start_time - ros::Time::now()).toSec() < 2.0)
    {
      geometry_msgs::Vector3 desired_vel;
      desired_vel.x = 0.5; desired_vel.y = desired_vel.z = 0;
      double desired_yaw = data.rpydata.z;//Current yaw
      parserinstance->cmdvelguided(desired_vel, desired_yaw);
      return;
    }

    rpytcmd.x = parsernode::common::map(data.servo_in[0],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.y = parsernode::common::map(data.servo_in[1],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.w = parsernode::common::map(data.servo_in[2],-10000, 10000, 10, 100);

    double yaw_rate = parsernode::common::map(data.servo_in[3],-10000, 10000, -M_PI, M_PI);
    rpytcmd.z = rpytcmd.z - yaw_rate*0.02;//50Hz
    if(rpytcmd.z > M_PI)
      rpytcmd.z = rpytcmd.z - 2*M_PI;
    else if(rpytcmd.z < -M_PI)
      rpytcmd.z = rpytcmd.z + 2*M_PI;

    //ROS_INFO("Timer Running");
    parserinstance->cmdrpythrust(rpytcmd, true);
  }
}

void OnboardNodeHandler::velcmdtimerCallback(const ros::TimerEvent& event)
{
  //Check if roi has not been updated for more than 0.5 sec; Then disable tracking automatically:
  if(enable_tracking)
  {
    parserinstance->getquaddata(data);
    bool result = roi_vel_ctrlr_->set(data.rpydata,desired_vel, desired_yaw);
    if(!result)
    {
        stateTransitionTracking(false);
        return;
    }
  }
  //send command of the velocity
  if(parserinstance)
    parserinstance->cmdvelguided(desired_vel, desired_yaw);
}

void OnboardNodeHandler::poscmdtimerCallback(const ros::TimerEvent& event)
{
  if(enable_poscontrol)
  {
  /*  goal_position.z = goal_altitude;
    //Position Interpolation
    tf::Vector3 goal_diff(goal_position.x - data.localpos.x, goal_position.y - data.localpos.y, goal_position.z - data.localpos.z);
    geometry_msgs::Vector3 current_goal;
    if(goal_diff.length() > waypoint_vel)
      goal_diff = waypoint_vel*goal_diff.normalize();
    tf::vector3TFToMsg(goal_diff, current_goal);
    current_goal.x += data.localpos.x; current_goal.y += data.localpos.y; current_goal.z += data.localpos.z;
    //Yaw Interpolation:
    double goal_yaw_diff = (desired_yaw - data.rpydata.z);
    if(goal_yaw_diff > waypoint_yawvel)
      goal_yaw_diff = waypoint_yawvel;
    else if(goal_yaw_diff < -waypoint_yawvel)
      goal_yaw_diff = -waypoint_yawvel;

*/
    /*if(std::abs(goal_yaw_diff) > waypoint_yawvel*0.02)
      goal_yaw_diff = std::copysign(waypoint_yawvel*0.02,goal_yaw_diff);
      */
 //   double current_desired_yaw = data.rpydata.z + goal_yaw_diff;
    goal_position.z = goal_altitude;
    parserinstance->cmdwaypoint(goal_position, desired_yaw);
  }
}

void OnboardNodeHandler::trajectorytimerCallback(const ros::TimerEvent& event)
{
    if(!enable_trajectory_tracking)
      return;

    //Follow the trajectory i.e set goal for quadcopter, set goals for arm based on current time and when the trajectory was req
    ros::Duration time_offset = ros::Time::now() - gcop_trajectory_request_time;
    cout<<"Time Offset: "<<time_offset<<endl;//[DEBUG]
    //Find matching nearest time in the trajectory:
    for(int count_timesearch = nearest_index_gcop_trajectory; count_timesearch <= gcop_trajectory.N ; count_timesearch++)
    {
      double tdiff = (gcop_trajectory.time[count_timesearch] - time_offset.toSec());
      if( tdiff > 0)
      {
        assert(count_timesearch != 0);//Since timeoffset is greater than zero and initial time in gcop ts is zero, this should not ideally happen
        nearest_index_gcop_trajectory = count_timesearch;//[NOT USING NEAREST TIME BUT GIVING FUTURE CLOSEST TIME]
        break;
      }
    }
    gcop_comm::State &goalstate = gcop_trajectory.statemsg[nearest_index_gcop_trajectory];
    parserinstance->getquaddata(data);//Get Latest data
    //Create a velocity command based on goal state:
    geometry_msgs::Vector3 desired_velocity_traj_tracking;
    desired_velocity_traj_tracking.x = goalstate.basetwist.linear.x + kp_trajectory_tracking*(goalstate.basepose.translation.x - data.localpos.x);
    desired_velocity_traj_tracking.y = goalstate.basetwist.linear.y + kp_trajectory_tracking*(goalstate.basepose.translation.y - data.localpos.y);
    desired_velocity_traj_tracking.z = goalstate.basetwist.linear.z + kp_trajectory_tracking*(goalstate.basepose.translation.z - data.localpos.z);
    double desired_yaw_traj_tracking = tf::getYaw(goalstate.basepose.rotation);
    
    //Send command
    parserinstance->cmdvelguided(desired_velocity_traj_tracking, desired_yaw_traj_tracking);

    if(time_offset.toSec() > gcop_trajectory.time[gcop_trajectory.N]+timeout_trajectory_tracking)
    {
        trajectorytimer.stop();
        stateTransitionTrajectoryTracking(false);
    }
}

void OnboardNodeHandler::mpctimerCallback(const ros::TimerEvent& event)
{
  if(enable_mpccontrol)//SAFETY
  {
    if(!mpc_closed_loop_)
    {
      //ROS_INFO("Number of controls: %d, %d",model_control.us.size(), mpc_trajectory_count);
      //OPENLOOP VERSION:
      if(mpc_trajectory_count < model_control.us.size())
      {
        Eigen::Vector4d &current_u = model_control.us.at(mpc_trajectory_count);
        gcop::QRotorIDState &current_x = model_control.xs.at(mpc_trajectory_count+1);
        rpytcmd.x = current_x.u[0];
        rpytcmd.y = current_x.u[1];
        rpytcmd.w = (current_u[0]>80)?80:(current_u[0]<20)?20:current_u[0];//Simple Bounds so we dont send crazy values
        rpytcmd.z = current_x.u[2];
        mpc_trajectory_count++;
        parserinstance->cmdrpythrust(rpytcmd, true);
        static ros::Time prev_time = ros::Time::now();
        //ROS_INFO("Current Control: %d, %f,%f,%f,%f,%f",mpc_trajectory_count, rpytcmd.x, rpytcmd.y, rpytcmd.z, rpytcmd.w, (ros::Time::now() - prev_time).toSec());
        //ROS_INFO("state.u[2]: %f,%f,%f",model_control.xs[0].u[2], model_control.xs[1].u[2], model_control.us[0][3]);
        //ROS_INFO("Current rate: %f,%f,%f",current_u[1], current_u[2], current_u[3]);
        prev_time = ros::Time::now();
      }
      else
      {
        mpctimer.stop();
        stateTransitionMPCControl(false);
      }
    }
    else
    {
      //CLOSEDLOOP VERSION:
      setInitialStateMPC();//Set Initial State
      //Check if initial state is close to goal position and quit:
      Eigen::Vector3d error_pos = (model_control.xf.p - model_control.xs[0].p);
      ros::Duration time_offset = (ros::Time::now() - mpc_request_time);
      ROS_INFO("ERROR Norm: %f, time_offset: %f", error_pos.norm(), time_offset.toSec()); //DEBUG
      if(error_pos.norm() < goal_tolerance || time_offset.toSec() > model_control.tf+ timeout_mpc_control)
      {
        mpctimer.stop();
        stateTransitionMPCControl(false);
      }
      else
      {
        model_control.iterate(Nit);//Iterate
        Eigen::Vector4d &current_u = model_control.us[0];//Get Current Control
        gcop::QRotorIDState &current_x = model_control.xs[1];
        rpytcmd.x = current_x.u[0];
        rpytcmd.y = current_x.u[1];
        rpytcmd.w = (current_u[0]>80)?80:(current_u[0]<20)?20:current_u[0];//Simple Bounds so we dont send crazy values
        rpytcmd.z = current_x.u[2];//TODO: Add Provision to send yaw cmd instead of yaw rate
        //If Needed do this. Iterate on a separate thread and send the closest state etc
        parserinstance->cmdrpythrust(rpytcmd, true);
      }
    }
  }
}
