#include <rqt_quadcoptergui/onboardnodehandler.h>
#include <tf_conversions/tf_eigen.h>
#define ARM_ENABLED
//#define ARM_MOCK_TEST_DEBUG

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh_):nh(nh_)
                                                            , broadcaster(new tf::TransformBroadcaster())
                                                            , logdir_created(false), enable_logging(false)
                                                            , publish_rpy(false)
                                                            , publish_vel(false)
                                                            , enable_tracking(false), enable_velcontrol(false), enable_rpytcontrol(false), enable_poscontrol(false)
                                                            , enable_mpccontrol(false), enable_trajectory_tracking(false)
                                                            , reconfig_init(false), reconfig_update(false)
                                                            , desired_yaw(0), kp_trajectory_tracking(1.0), timeout_trajectory_tracking(1.0), timeout_mpc_control(1.0)
                                                            , model_control(nh_, "world"), mpc_closed_loop_(false), mpc_trajectory_count(0)
                                                            , so3(SO3::Instance()), iterate_mpc_thread(NULL), mpc_thread_iterating(false)
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

  ROS_INFO("Creating arm instance");
  createArmInstance();

  //Create RoiVelController:
  if(!use_alvar_)
  {
    roi_vel_ctrlr_.reset(new RoiVelController(nh,uav_name));
  }
  else
  {
    ROS_INFO("Using Alvar Tracking");
    roi_vel_ctrlr_.reset(new AlvarTrackController(nh,uav_name));
  }
  roi_vel_ctrlr_->setCameraTransform(CAM_QUAD_transform);
  nh.param<double>("/control/obj_dist_max", roi_vel_ctrlr_->obj_dist_max,2.0);

  //Create Velocity controller:
  vel_ctrlr_.reset(new QuadVelController(delay_send_time_));

/*#ifdef ARM_ENABLED
  ROS_INFO("Creating Arm Hardware Instance");
  arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
#endif */

  ROS_INFO("Subscribing to Callbacks");
  //Subscribe to GuiCommands
  gui_command_subscriber_ = nh.subscribe("/gui_commands", 10, &OnboardNodeHandler::receiveGuiCommands, this);
  goal_pose_subscriber_ = nh.subscribe("goal",1,&OnboardNodeHandler::receiveGoalPose,this);
  trajectory_subscriber_ = nh.subscribe("ctrltraj",1,&OnboardNodeHandler::gcoptrajectoryCallback,this);
  guidance_obs_dist_ = nh.subscribe("/guidance/obstacle_distance",1,&OnboardNodeHandler::receiveObstacleDistance,this);
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
  //if(publish_vel)
  {
    global_vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("global_vel",10);
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
  armcmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::armcmdTimerCallback,this);
  armcmdtimer.stop();
  //Timer for commanding quadcopter
  velcmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::velcmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  velcmdtimer.stop();
  mpcveltimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::mpcveltimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  mpcveltimer.stop();
  poscmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::poscmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  poscmdtimer.stop();
  hometimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::poscmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  hometimer.stop();
  mpctimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::mpctimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  mpctimer.stop();
  rpytimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::rpytimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  rpytimer.stop();
  rpy_stop_timer = nh_.createTimer(ros::Duration(measurement_period), &OnboardNodeHandler::onlineOptimizeCallback,this, true);//One shot timer to stop rpy Callback after 10 seconds
  rpy_stop_timer.stop();
  trajectorytimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::trajectorytimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  trajectorytimer.stop();
  //Timer for sending quadcopter state
  quadstatetimer = nh_.createTimer(ros::Duration(0.05), &OnboardNodeHandler::quadstatetimerCallback, this);//10Hz update GUI quad state
}

OnboardNodeHandler::~OnboardNodeHandler()
{
  //Poweroff arm:
/*#ifdef ARM_ENABLED
  arm_hardwareinst->powermotors(false);
#endif
*/
  printf("I am called");
  arm_model.reset();
  ROS_INFO("Powering off arm");
  arm_hardware_controller_->powerOff();//Power off arm before exiting
  arm_hardware_controller_.reset();

  parserinstance.reset();
  parser_loader.reset();
  //ctrlrinst.reset();
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

inline void OnboardNodeHandler::createArmInstance()
{
  arm_model.reset(new gcop::Arm()); 
  arm_hardware_controller_.reset(new ArmHardwareController(nh));
  arm_hardware_controller_->foldArm();
  arm_hardware_controller_->setJointSpeeds(arm_default_speed_, arm_default_speed_, arm_default_speed_);
}

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
  initial_state_vel_.setZero();
  meas_filled_ = 0;
  mpc_delay_rpy_data.x = mpc_delay_rpy_data.y = mpc_delay_rpy_data.z =0;

  //Setup arm variables:
  Eigen::Affine3d cam_quad_tf_eig, arm_quad_tf_eig;
  tf::transformTFToEigen(CAM_QUAD_transform, cam_quad_tf_eig);
  tf::transformTFToEigen(ARM_QUAD_transform, arm_quad_tf_eig);
  arm_cam_tf_eig_ = arm_quad_tf_eig.inverse()*cam_quad_tf_eig;
  tolerance_tip_pos_ = 0.05;//5 cm

  //systemid_measurements.reserve(600);
  //control_measurements.reserve(600);
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
  nh.param<bool>("/gui/publishvel",publish_vel,false);
  nh.param<double>("/mpc/goal_tolerance", goal_tolerance,0.2);//Stop when 0.2m away from goal
  nh.param<bool>("/control/optimize_online", optimize_online_,false);
  nh.param<bool>("/control/set_offsets_mpc", set_offsets_mpc_,false);
  nh.param<double>("/control/measurement_period", measurement_period,10.0);
  nh.param<double>("/control/vel_send_time", vel_send_time_,3.0);
  nh.param<double>("/control/delay_send_time", delay_send_time_,0.2);
  nh.param<bool>("/control/virtual_obstacle", virtual_obstacle_,true);
  nh.param<double>("/control/offsets_timeperiod", systemid.offsets_timeperiod,0.5);
  nh.param<bool>("/control/use_alvar", use_alvar_,false);
  nh.param<double>("/arm/joint_speed",arm_default_speed_,0.6);//Default 0.6 rad/s
  {
    std::string systemid_filename;
    nh.getParam("/control/systemid_params",systemid_filename);
    gcop::loadParameters(systemid_filename,systemid);
  }

  ROS_INFO("Dummy Times: %f,%f",delay_send_time_, vel_send_time_);

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
  try{
    bool result = listener.waitForTransform(uav_name, "arm",
                                            ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform(uav_name, "arm",
                             ros::Time(0), ARM_QUAD_transform);
    if(!result)
      cout<<"Cannot find QUAD to ARM Transform"<<endl;
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

inline void OnboardNodeHandler::logMeasurements(bool mpc_flag)
{
  if(!logdir_created)
    setupLogDir();
  std::string filename = logdir_stamped_+"/measurements";

  if(mpc_flag)
      filename = filename + "mpc";
  filename = parsernode::common::addtimestring(filename);
  ofstream systemidfile(filename.c_str());
  systemidfile.precision(10);
  for(int i = 0; i < systemid_measurements.size(); i++)
  {
    QRotorSystemIDMeasurement &measurement = systemid_measurements[i];
    systemidfile<<measurement.t<<" "<<measurement.position.transpose()<<" "<<measurement.rpy.transpose()<<" "<<measurement.control.transpose()<<" "<<control_measurements[i].transpose()<<endl;
  }
  systemid_measurements.clear();//Clear vector for new measurements
  control_measurements.clear();
}


inline void OnboardNodeHandler::setupLogDir()
{
  // Logger
  std::string logdir_append = logdir + "/session";
  logdir_stamped_ = parsernode::common::addtimestring(logdir_append);
  ROS_INFO("Creating Log dir: %s",logdir_stamped_.c_str());
  int status = mkdir(logdir_stamped_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);//Create the directory see http://pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
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

  logdir_created = true;//Specify that log directory has been created
}

/*inline void OnboardNodeHandler::setInitialStateMPC()
{
  parserinstance->getquaddata(data);//Get Latest data
  model_control.setInitialState(data.localpos, data.linvel,
                                data.rpydata, data.omega, rpytcmd);
}
*/

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
      if(parserinstance)
        parserinstance->setlogdir(logdir_stamped_);
      parserinstance->controllog(true);
      roi_vel_ctrlr_->setlogdir(logdir_stamped_);
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
      //Clear buffers of vel controller:
      vel_ctrlr_->setGoal(desired_vel.x, desired_vel.y, desired_vel.z, desired_yaw);
      vel_ctrlr_->resetSmoothVel();
      vel_ctrlr_->clearBuffer();
      rpytcmd.x = rpytcmd.y = rpytcmd.z = 0;
      rpytcmd.w = (9.81/systemid.qrotor_gains(0));//Set to Default Value
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
      mpcveltimer.stop();
      mpctimer.stop();
      //Set current vel to 0:
      desired_vel.x = desired_vel.y = desired_vel.z = 0;
      desired_yaw = data.rpydata.z;
      vel_ctrlr_->setGoal(desired_vel.x, desired_vel.y, desired_vel.z, desired_yaw);
      vel_ctrlr_->resetSmoothVel();
      //parserinstance->cmdvelguided(desired_vel, desired_yaw);
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
        rpytcmd.x = rpytcmd.y = 0;//Set Commanded roll and pitch to 0
        rpytcmd.z = data.rpydata.z;//Current Yaw
        mpc_request_time = ros::Time::now();
        mpc_trajectory_count = 0;
        //Set Initial Object Velocity based on real or virtual obstacle
        /*if(!virtual_obstacle_)
        {
          geometry_msgs::Vector3 des_obj_dir;
          bool result = roi_vel_ctrlr_->setDesiredObjectDir(data.rpydata,des_obj_dir);
          if(!result)
          {
            ROS_WARN("Failed to initialize des obj dir");
            enable_mpccontrol = false;
            goto PUBLISH_MPC_CONTROL_STATE;
          }
          else
          {
            desired_yaw = atan2(des_obj_dir.y, des_obj_dir.x);
            ROS_INFO("Desired Yaw: %f",desired_yaw);
            //Get Desired Obj direction and Publish the marker
            visualization_msgs::Marker dirxn_marker;
            dirxn_marker.id = 2;
            dirxn_marker.header.frame_id = "world";
            dirxn_marker.action = visualization_msgs::Marker::ADD;
            dirxn_marker.type = visualization_msgs::Marker::ARROW;

            geometry_msgs::Point pt;
            copyPtToVec(data.localpos,pt);
            dirxn_marker.points.push_back(pt);

            pt.x += 2.0*des_obj_dir.x;
            pt.y += 2.0*des_obj_dir.y;
            pt.z += 2.0*des_obj_dir.z;
            dirxn_marker.points.push_back(pt);
            dirxn_marker.scale.x = 0.02;//Shaft dia
            dirxn_marker.scale.y = 0.05;//Head Dia
            dirxn_marker.color.r = 1.0;//Red arrow
            dirxn_marker.color.a = 1.0;
            marker_pub_.publish(dirxn_marker);
            //Set Initial Velocity:
            initial_state_vel_ = model_control.xs[0].v.norm()*Eigen::Vector3d(des_obj_dir.x, des_obj_dir.y, des_obj_dir.z);
            ROS_INFO("Initial State Vel: %f,%f,%f",initial_state_vel_[0],initial_state_vel_[1],initial_state_vel_[2]);
          }
        }
        else
        */
        {
          desired_yaw = data.rpydata.z;
          //Set Initial State Velocity Desired:
          const QRotorIDState &x0 = model_control.getInitialState();
          Matrix3d yawM;
          Vector3d rpy(0,0,data.rpydata.z);
          so3.q2g(yawM, rpy);
          initial_state_vel_ = yawM*x0.v; 
        }
        //Clear iterate mpc thread:
        iterate_mpc_thread = NULL;

        //Reset vel ctrlr smoothness
        vel_ctrlr_->setGoal(0,0,0, desired_yaw);
        vel_ctrlr_->resetSmoothVel();
        //Clear buffers of vel controller:
        vel_ctrlr_->clearBuffer();
        rpytcmd.x = rpytcmd.y = rpytcmd.z = 0;
        rpytcmd.w = (9.81/systemid.qrotor_gains(0));//Set to Default Value
        //Start MPC Vel Timer to achieve initial vel and start mpctimerCallback
        mpcveltimer.start();
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
  if(enable_rpytcontrol)
    stateTransitionRpytControl(false);
  if(enable_tracking)
    stateTransitionTracking(false);
  if(enable_velcontrol)
    stateTransitionVelControl(false);
  if(enable_poscontrol)
    stateTransitionPosControl(false);
  if(enable_mpccontrol)
    stateTransitionMPCControl(false);

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

inline void OnboardNodeHandler::stateTransitionEnableArm(bool state)
{
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.commponent_id = state_message.enable_arm_status;
  state_message.status = enable_arm;

  //TODO If vel tracking is enabled, disable it.

  if(!arm_model || !arm_hardware_controller_)
  {
    ROS_WARN("Arm model not loaded. Cannot enable arm");
    gui_state_publisher_.publish(state_message);
    return;
  }

  if(state)
  {
    armcmdtimer.start();
    enable_arm = true;
  }
  else
  {
    enable_arm = false;
    //Stop Trajectory tracking timer:
    armcmdtimer.stop();
  }
  //Publish Change of State:
  state_message.status = enable_arm;
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
      //Online Optimization Settings
      prev_ctrl_time = 0;// Record when previous control was sent
      prev_rp_cmd[0] = prev_rp_cmd[1] = 0;//Set initial commands to 0;
      parserinstance->getquaddata(data);
      //model_control.setInitialState(data.localpos,data.linvel,data.rpydata,data.omega,rpytcmd,systemid_init_state);//Set Initial State for SystemID
      meas_filled_ = 0;
      rpytimer.start();

      if(optimize_online_)
      {
          ROS_INFO("Starting Online Optimize Timer");
          rpy_stop_timer.setPeriod(ros::Duration(measurement_period));
          rpy_stop_timer.start();
      }
      enable_rpytcontrol = true;
    }
    else
    {
      ROS_INFO("Stopping rpy timer");
      rpytimer.stop();
      enable_rpytcontrol = false;
      //Set current vel to 0:
      //desired_vel.x = desired_vel.y = desired_vel.z = 0;
      //desired_yaw = data.rpydata.z;
      //parserinstance->cmdvelguided(desired_vel, desired_yaw);
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
  //model_control.setInitialState(data.linvel);
  model_control.iterate();
  //Log MPC Trajectory

  if(!logdir_created)
    setupLogDir();
  {

    std::string filename = logdir_stamped_+"/mpctrajectory";
    filename = parsernode::common::addtimestring(filename);
    model_control.logTrajectory(filename);
  }

  model_control.publishTrajectory(data.localpos, data.rpydata);
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
  case command_msg.enable_arm:
    stateTransitionEnableArm(command_msg.command);
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
}

void OnboardNodeHandler::receiveObstacleDistance(const sensor_msgs::LaserScan &scan)
{
    double obs_dist = scan.ranges[1];
    //ROS_INFO("Received Obstacle dist: %f",obs_dist);//DEBUG
    tf::Transform obs_transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(obs_dist,0,0));
    broadcaster->sendTransform(tf::StampedTransform(obs_transform, ros::Time::now(), uav_name, "obs"));
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

    if(enable_logging)
    {
        std::string trajfilename = 
          parsernode::common::addtimestring(logdir_stamped_+"/traj_tracking_data") + ".dat";
        trajfile.open(trajfilename.c_str());
        if(!trajfile.is_open())
        {
            ROS_WARN("Could not open %s for trajectory logging", 
              trajfilename.c_str()); 
        }
    }
    
    nearest_index_gcop_trajectory = 0;//Every time we get a new trajectory we reset the time index for searching nearest current time to zero
    gcop_trajectory_request_time = ros::Time::now();//Record when we received request
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
    config.mpc_velmag = model_control.x0().v.norm();
    /*config.mpc_goalx = model_control.xf.p[0];
    config.mpc_goaly = model_control.xf.p[1];
    config.mpc_goalz = model_control.xf.p[2];
    config.mpc_goalyaw = so3.yaw(model_control.xf.R);
    */
    config.delay_send_time = delay_send_time_;
    //config.kp_velctrl = vel_ctrlr_->kp_;
    //config.ki_velctrl = vel_ctrlr_->ki_;
    //vel_ctrlr_->kp_ = config.kp_velctrl;
    //vel_ctrlr_->ki_ = config.ki_velctrl;
    //cout<<"Goal Yaw: "<<config.mpc_goalyaw<<endl;
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
  //roi_vel_ctrlr_->setGains(config.radial_gain, config.tangential_gain, config.desired_object_distance);
  goal_altitude = config.goal_altitude;
  Nit = config.Nit;
  mpc_closed_loop_ = config.mpc_closed_loop;
  kp_trajectory_tracking = config.kp_trajectory_tracking;
  //Set Goal for MPC:
  /*if(model_control.xs[0].v.norm() > 0.01)//At least 1 cm/s Then only scale it otherwise no effect of velmag
  {
    model_control.xs[0].v = (config.mpc_velmag/model_control.xs[0].v.norm())*model_control.xs[0].v;
    model_control.xf.v = (config.mpc_velmag/model_control.xf.v.norm())*model_control.xf.v;//Same magnitude as final
  }
  */
  //model_control.xs[0].v[0] = config.mpc_vel;
  /*model_control.xf.p[0] = config.mpc_goalx;
  model_control.xf.p[1] = config.mpc_goaly;
  model_control.xf.p[2] = config.mpc_goalz;
  Vector3d rpy(0,0,config.mpc_goalyaw);
  so3.q2g(model_control.xf.R,rpy);
  */
  if(config.reset_controls)
  {
      model_control.resetControls();//Reset the controls to base value when reset is pressed
      config.reset_controls = false;//Set back to false
  }
  if(config.go_home)
  {
      bool result = parserinstance->flowControl(true);//get control
      if(result)
      {
        ROS_INFO("Home Timer Starting");
        home_start_time = ros::Time::now();
        hometimer.start();
      }
      config.go_home = false;
  }
  if(config.record_home)
  {
    goal_position.x = data.localpos.x;
    goal_position.y = data.localpos.y;
    goal_position.z = data.localpos.z;
    desired_yaw = data.rpydata.z;
    ROS_INFO("Recorded Pos: %f,%f,%f; Yaw: %f", goal_position.x, goal_position.y, goal_position.z, desired_yaw);
    config.record_home = false;
  }
  delay_send_time_ = config.delay_send_time;
  if(config.power_off_arm)
  {
      config.power_off_arm = false;
      if(arm_hardware_controller_)
          arm_hardware_controller_->powerOff();
  }
  if(config.fold_arm)
  {
      config.fold_arm = false;
      if(arm_hardware_controller_)
          arm_hardware_controller_->foldArm();
  }
  vel_ctrlr_->smooth_fac_ = config.smooth_fac_velctrl;
  vel_ctrlr_->kp_ = config.kp_velctrl;
  vel_ctrlr_->ki_[0] = vel_ctrlr_->ki_[1] = config.ki_velctrlxy;
  vel_ctrlr_->ki_[2] = config.ki_velctrlz;
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
  //if(publish_vel)
  {
    geometry_msgs::Vector3 global_vel_msg;
    global_vel_msg.x = data.linvel.x;
    global_vel_msg.y = data.linvel.y;
    global_vel_msg.z = data.linvel.z;
    global_vel_pub_.publish(global_vel_msg);
  }
  //Convert data servo_in to rpytcmd:
  if(!enable_rpytcontrol && !enable_mpccontrol && !enable_velcontrol)
  {
    rpytcmd.x = parsernode::common::map(data.servo_in[0],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.y = parsernode::common::map(data.servo_in[1],-10000, 10000, -M_PI/6, M_PI/6);
    rpytcmd.w = parsernode::common::map(data.servo_in[2],-10000, 10000, 10, 100);
    rpytcmd.z = data.rpydata.z;//Current yaw
    //rpytcmd.z = parsernode::common::map(data.servo_in[3],-10000, 10000, -M_PI, M_PI);
  }

 /* geometry_msgs::Vector3 object_position_cam_geo;
  tf::Vector3 object_position_quad(0,0,0);
  if( roi_vel_ctrlr_->getObjectPosition(object_position_cam_geo))
  {
    object_position_quad = tf::Vector3(object_position_cam_geo.x, object_position_cam_geo.y, object_position_cam_geo.z);
    tf::Matrix3x3 quad_orientation;
    quad_orientation.setEulerYPR(0, data.rpydata.y, data.rpydata.x);
    double cyaw = cos(data.rpydata.z);
    double syaw = sin(data.rpydata.z);
    object_position_quad = quad_orientation*(CAM_QUAD_transform*object_position_quad) - tf::Vector3(data.linvel.x*cyaw +data.linvel.y*syaw, -data.linvel.x*syaw+data.linvel.y*cyaw, data.linvel.z)*2*delay_send_time_;//Get Object Position in Quad frame
  }
*/
  double object_distance = roi_vel_ctrlr_->getObjectDistance();
  // Create a Text message based on the data from the Parser class
  sprintf(buffer,
          "Battery Percent: %2.2f\t\nlx: %2.2f\tly: %2.2f\tlz: %2.2f\nAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\n"
          "Magx: %2.2f\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz %2.2f\n"
          "Trvelx: %2.2f\tTrvely: %2.2f\tTrvelz: %2.2f\tTrObjD: %2.2f\nCmdr: %2.2f\tCmdp: %2.2f\tCmdt: %2.2f\tCmdy: %2.2f\n"
          "Goalx: %2.2f\tGoaly: %2.2f\tGoalz: %2.2f\tGoaly: %2.2f\n"
  //        "Objx: %2.2f\tObjy: %2.2f\tObjz: %2.2f\n"
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
 //         ,object_position_quad[0], object_position_quad[1], object_position_quad[2]
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

    //if(test_vel)
    //{
      /*if((ros::Time::now() - rpytimer_start_time).toSec() < 2.0)///Only for checking if we can switch between Velocity and RPY Modes [REMOVE LATER]
      {
        geometry_msgs::Vector3 desired_vel;
        desired_vel.x = -5; desired_vel.y = desired_vel.z = 0;
        double desired_yaw = data.rpydata.z;//Current yaw
        parserinstance->cmdvelguided(desired_vel, desired_yaw);
        return;
      }
      */
    //}
    /*if((event.current_real - rpytimer_start_time).toSec() < 0.2)
    {
      rpytcmd.x = rpytcmd.y = 0;
      rpytcmd.z = data.rpydata.z;//Set to current yaw
      rpytcmd.w = (9.81/systemid.qrotor_gains(0));//Set to Default Value
      parserinstance->cmdrpythrust(rpytcmd, true);
      ROS_INFO("Sending zero rpy: %f,%f,%f, %f",rpytcmd.x, rpytcmd.y, rpytcmd.z, rpytcmd.w);
      return;
    }*/

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

    if(optimize_online_)
    {
        double time_ = (ros::Time::now() - rpytimer_start_time).toSec();
        if(time_ >= delay_send_time_ && meas_filled_< systemid_measurements.size())
        {
          QRotorSystemIDMeasurement &prev_meas = systemid_measurements[meas_filled_];
          prev_meas.position<<data.localpos.x, data.localpos.y, data.localpos.z;
          prev_meas.rpy<<data.rpydata.x, data.rpydata.y, data.rpydata.z;
          meas_filled_++;
          //ROS_INFO("Meas Filled: %d, Size: %d",meas_filled_, systemid_measurements.size());
        }

        if(time_ <= measurement_period - delay_send_time_)
        {
          QRotorSystemIDMeasurement measurement;
          measurement.t = time_;
          double time_diff = measurement.t - prev_ctrl_time;
          if(time_diff < 0.005)
          {
            ROS_WARN("Time diff too small Expected 0.02; Found: %f",time_diff);
            time_diff = 0.005;//Hard reset;
          }

          measurement.control[0] = rpytcmd.w;
          measurement.control[1] = (rpytcmd.x - prev_rp_cmd[0])/time_diff;
          measurement.control[2] = (rpytcmd.y - prev_rp_cmd[1])/time_diff;
          measurement.control[3] = -yaw_rate;
          //Copy over previous values:
          prev_rp_cmd[0] = rpytcmd.x;
          prev_rp_cmd[1] = rpytcmd.y;
          prev_ctrl_time = measurement.t;
          //Record:
          systemid_measurements.push_back(measurement);
          control_measurements.push_back(Vector3d(rpytcmd.x, rpytcmd.y, rpytcmd.z));
        }
    }
  }
}

void OnboardNodeHandler::onlineOptimizeCallback(const ros::TimerEvent &event)
{
    //Stop rpytControl:
    stateTransitionRpytControl(false);

    while(meas_filled_ < systemid_measurements.size())
    {
        systemid_measurements.pop_back();
    }

    Matrix7d stdev_gains;
    Vector6d mean_offsets;
    Matrix6d stdev_offsets;
    QRotorIDState systemid_init_state;
    systemid_init_state.Clear();
    systemid_init_state.p = systemid_measurements[0].position;
    so3.q2g(systemid_init_state.R,systemid_measurements[0].rpy);
    systemid_init_state.u<<0,0,systemid_measurements[0].rpy(2);
    systemid.EstimateParameters(systemid_measurements,systemid_init_state,&stdev_gains, &mean_offsets, &stdev_offsets);//Estimate Parameters
    systemid.qrotor_gains[0] -= 0.004;
    if(!set_offsets_mpc_)
    {
      model_control.setParametersAndStdev(systemid.qrotor_gains,stdev_gains);//Set Optimization to right gains
      vel_ctrlr_->setParameters(systemid.qrotor_gains);
    }
    else
    {
      model_control.setParametersAndStdev(systemid.qrotor_gains,stdev_gains,&mean_offsets,&stdev_offsets);//Set Optimization to right gains
      vel_ctrlr_->setParameters(systemid.qrotor_gains, &mean_offsets);
    }
    //Iterate through fixed MPC Problem
    //model_control.iterate();

    /*model_control.iterate();
    //Log MPC Trajectory

    if(!logdir_created)
      setupLogDir();
    {

      std::string filename = logdir_stamped_+"/mpctrajectory";
      filename = parsernode::common::addtimestring(filename);
      model_control.logTrajectory(filename);
    }
    */

    //Print all measurements:
    logMeasurements(false);
    {
      //Log the optimal estimation parameters:
      std::string filename = logdir_stamped_+"/estimationparams";
      filename = parsernode::common::addtimestring(filename);
      ofstream systemidparamfile(filename.c_str());
      systemidparamfile.precision(10);
      systemidparamfile<<"Gains: "<<systemid.qrotor_gains.transpose()<<endl<<"Stdev Gains: "<<endl<< stdev_gains<< endl<<" Mean Offsets: "<<mean_offsets.transpose()<<endl<<"Stdev Offsets: "<<endl<<stdev_offsets<<endl;
    }
}

void OnboardNodeHandler::getCurrentState(QRotorIDState &state)
{
  //Fill current state:
  state.p<<data.localpos.x, data.localpos.y, data.localpos.z;
  state.v<<data.linvel.x, data.linvel.y, data.linvel.z;
  Vector3d rpy(data.rpydata.x, data.rpydata.y, data.rpydata.z);
  so3.q2g(vel_ctrlr_state.R, rpy);
  state.w<<data.omega.x, data.omega.y, data.omega.z;
}

void OnboardNodeHandler::velcmdtimerCallback(const ros::TimerEvent& event)
{
  parserinstance->getquaddata(data);
  getCurrentState(vel_ctrlr_state);
  //vel_ctrlr_state.u<<rpytcmd.x, rpytcmd.y, rpytcmd.z;
  //Check if roi has not been updated for more than 0.5 sec; Then disable tracking automatically:
  if(enable_tracking)
  {
    bool result = roi_vel_ctrlr_->set(data.rpydata,desired_vel, desired_yaw);
    if(!result)
    {
        stateTransitionTracking(false);
        return;
    }
  }
  //Set desired vel:
  vel_ctrlr_->setGoal(desired_vel.x, desired_vel.y, desired_vel.z, desired_yaw);
  //Find command rpy for desired velocity:
  Vector4d command;
  vel_ctrlr_->set(vel_ctrlr_state, command);
  rpytcmd.x = command[1];
  rpytcmd.y = command[2];
  rpytcmd.z = command[3];
  rpytcmd.w = command[0];
  //send command rpy
  if(parserinstance)
    parserinstance->cmdrpythrust(rpytcmd, true);
    //parserinstance->cmdvelguided(desired_vel, desired_yaw);
}

void OnboardNodeHandler::armcmdTimerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("arm callback");
  if(enable_arm && enable_tracking)
  {  
    geometry_msgs::Vector3 object_position_cam_geo;
    if(!roi_vel_ctrlr_->getObjectPosition(object_position_cam_geo))
    {
      ROS_WARN("Unable to get object position!");
      return;
    }
    // transform object position from camera to arm frame
    Eigen::Vector3d object_position_cam(object_position_cam_geo.x, object_position_cam_geo.y, object_position_cam_geo.z);
    Eigen::Vector3d object_position_arm = arm_cam_tf_eig_*object_position_cam;
    if(arm_model)
    {
      double a[2][3];
      double p[3] = {object_position_arm(0), 0, object_position_arm(2)};
      
      double ikres = arm_model->Ik(a, p);
      if(ikres > 0)//The object is within reach
      {
        arm_hardware_controller_->setJointAngles(a[0][1],a[0][2],1700);
      }
      //Check if the tip is close to the object:
      double angles_arm[3] = {0, (*arm_hardware_controller_)[0], (*arm_hardware_controller_)[1]};
      double tip_pos[3];
      arm_model->Fk(tip_pos, angles_arm);
      double error_tip_pos = pow((tip_pos[0] - p[0]),2) + pow((tip_pos[2] - p[2]),2);
      if(error_tip_pos < tolerance_tip_pos_)
      {
        ROS_INFO("Closing Gripper");
        arm_hardware_controller_->setGripperAngle(1200);
        //Timeout for few secs and 
        ros::Duration(1.0).sleep();//sleep for a second
        //Retract Arm
        arm_hardware_controller_->foldArm();
        //   TODO: Disable Arm Tracking
        armcmdtimer.stop();//Stop this timer.
        stateTransitionEnableArm(false);
        //   TODO: Add Logging of tip position and commanded tip position and error
      }
    }
    else
    {
      ROS_WARN("No arm instance!");
    }
  }
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
  else//Home Timer
  {
    //ROS_INFO("Goal Posn: %f,%f,%f, %f", goal_position.x, goal_position.y, goal_position.z, desired_yaw);
    parserinstance->cmdwaypoint(goal_position, desired_yaw);
    if((event.current_real - home_start_time).toSec()> 10)
    {
        ROS_INFO("Stopping Pos Timer");
        hometimer.stop();
    }
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

    if(enable_logging && trajfile.is_open())
    {
        trajfile << time_offset << " " 
          << goalstate.basepose.translation.x << " "
          << goalstate.basepose.translation.y << " "
          << goalstate.basepose.translation.z << " "
          << tf::getYaw(goalstate.basepose.rotation) << " "
          << goalstate.basetwist.linear.x << " "
          << goalstate.basetwist.linear.y << " "
          << goalstate.basetwist.linear.z << " "
          << data.localpos.x << " "
          << data.localpos.y << " "
          << data.localpos.z << " "
          << desired_velocity_traj_tracking.x << " "
          << desired_velocity_traj_tracking.y << " "
          << desired_velocity_traj_tracking.z << " "
          << data.rpydata.x << " "
          << data.rpydata.y << " "
          << data.rpydata.z << " "
          << std::endl;
    }

    
    //Send command
    parserinstance->cmdvelguided(desired_velocity_traj_tracking, desired_yaw_traj_tracking);

    if(time_offset.toSec() > gcop_trajectory.time[gcop_trajectory.N]+timeout_trajectory_tracking)
    {
        if(trajfile.is_open())
          trajfile.close();
        trajectorytimer.stop();
        stateTransitionTrajectoryTracking(false);
    }
}

void OnboardNodeHandler::iterateMPC()///Used by boost thread
{
  mpc_thread_mutex.lock();
  mpc_thread_iterating = true;
  mpc_thread_mutex.unlock();
  model_control.iterate(true);//Fast Iterate
  //Log Trajectory
  if(!logdir_created)
    setupLogDir();
  {

    std::string filename = logdir_stamped_+"/mpctrajectory";
    filename = parsernode::common::addtimestring(filename);
    model_control.logTrajectory(filename);
  }
  mpc_thread_mutex.lock();
  ROS_INFO("Iterating done");
  mpc_thread_iterating = false;
  mpc_thread_mutex.unlock();
}

void OnboardNodeHandler::mpcveltimerCallback(const ros::TimerEvent & event)
{
  //cout<<"Obstacle Dist: "<<model_control.getDesiredObjectDistance(delay_send_time_);//DEBUG
  //ROS_INFO("Vel sent: %f,%f,%f",desired_vel.x, desired_vel.y, desired_vel.z);
  parserinstance->getquaddata(data);

  if(virtual_obstacle_)
  {
    double timediff = (event.current_real - mpc_request_time).toSec();
    
    /*if(timediff >= vel_send_time_-0.3 && !iterate_mpc_thread)//Time for Optimizing
    {
      //Start Iterating MPC:
      ROS_INFO("Initial Vel: %f,%f,%f",data.linvel.x, data.linvel.y, data.linvel.z);
      model_control.setInitialState(data.linvel, data.rpydata);
      ROS_INFO("Starting MPC Thread");
      iterate_mpc_thread = new boost::thread(boost::bind(&OnboardNodeHandler::iterateMPC,this));//Start Iterating 0.3 seconds before rpyt mode is started
    }
    else */
    if(timediff >= vel_send_time_)//First 3 seconds
    {
        mpcveltimer.stop();
        mpc_request_time = event.current_real;
        mpc_delay_rpy_data = data.rpydata;
        ROS_INFO("Starting mpc timer");
        mpctimer.start();
        //parserinstance->getquaddata(data);
        mpc_delay_rpy_data = data.rpydata;
        initial_state_vel_<<data.linvel.x, data.linvel.y, data.linvel.z;
        model_control.setInitialState(data.linvel, data.rpydata);

        ROS_INFO("Initial Vel: %f,%f,%f",data.linvel.x, data.linvel.y, data.linvel.z);
        ROS_INFO("Starting MPC Thread");
        iterate_mpc_thread = new boost::thread(boost::bind(&OnboardNodeHandler::iterateMPC,this));//Start Iterating only one run
    }
  }
  else
  {
    //Set Desired distance based on recomputed roi position
      //geometry_msgs::Vector3 des_obj_dir;
      //bool result = roi_vel_ctrlr_->setDesiredObjectDir(data.rpydata,des_obj_dir);
      //desired_yaw = atan2(des_obj_dir.y, des_obj_dir.x);
      //initial_state_vel_ = model_control.xs[0].v.norm()*Eigen::Vector3d(des_obj_dir.x, des_obj_dir.y, des_obj_dir.z);

      
      //double object_dist = roi_vel_ctrlr_->getObjectDistance();

      geometry_msgs::Vector3 object_position_cam_geo;
      if( roi_vel_ctrlr_->getObjectPosition(object_position_cam_geo))
      {
        tf::Vector3 object_position_quad(object_position_cam_geo.x, object_position_cam_geo.y, object_position_cam_geo.z);
        tf::Matrix3x3 quad_orientation;
        quad_orientation.setEulerYPR(0, data.rpydata.y, data.rpydata.x);
        double cyaw = cos(data.rpydata.z);
        double syaw = sin(data.rpydata.z);
        object_position_quad = quad_orientation*(CAM_QUAD_transform*object_position_quad) - tf::Vector3(data.linvel.x*cyaw +data.linvel.y*syaw, -data.linvel.x*syaw+data.linvel.y*cyaw, data.linvel.z)*2*delay_send_time_;//Get Object Position in Quad frame
        double xydist_to_obs = tf::Vector3(object_position_quad[0], object_position_quad[1], 0).length();//Make this based on obstacle axis etc
        //ROS_INFO("xydist_to_obs: %f",xydist_to_obs);
        double linvel = tf::Vector3(data.linvel.x, data.linvel.y, data.linvel.z).length();
        if(xydist_to_obs < model_control.getDesiredObjectDistance(0) && linvel > 0.3)
        {
          mpcveltimer.stop();
          mpc_request_time = event.current_real;
          ROS_INFO("Starting mpc timer: %f", xydist_to_obs);
          mpctimer.start();

          model_control.setObstacleCenter(0,object_position_quad[0], object_position_quad[1], object_position_quad[2]);

          //parserinstance->getquaddata(data);
          mpc_delay_rpy_data = data.rpydata;
          model_control.setInitialState(data.linvel, data.rpydata);
          initial_state_vel_<<data.linvel.x, data.linvel.y, data.linvel.z;
          ROS_INFO("Initial Vel: %f,%f,%f",data.linvel.x, data.linvel.y, data.linvel.z);
          ROS_INFO("Object Position Quad: %f,%f,%f",object_position_quad[0], object_position_quad[1], object_position_quad[2]);
          ROS_INFO("Starting MPC Thread");
          iterate_mpc_thread = new boost::thread(boost::bind(&OnboardNodeHandler::iterateMPC,this));//Start Iterating only one run
        }
      }
  }

  //Set desired vel:
  vel_ctrlr_->setGoal(initial_state_vel_[0], initial_state_vel_[1], initial_state_vel_[2], desired_yaw);
  //Get current state
  getCurrentState(vel_ctrlr_state);
  //Find command rpy for desired velocity:
  Vector4d command;
  vel_ctrlr_->set(vel_ctrlr_state, command);
  rpytcmd.x = command[1];
  rpytcmd.y = command[2];
  rpytcmd.z = command[3];
  rpytcmd.w = command[0];
  //send command rpy to get desired vel
  parserinstance->cmdrpythrust(rpytcmd,true);
}

void OnboardNodeHandler::mpctimerCallback(const ros::TimerEvent& event)
{
  //For first 0.2 seconds send zero controls The quadcopter is responding only after that:
  if((event.current_real - mpc_request_time).toSec() < delay_send_time_)
  {
    rpytcmd.x = mpc_delay_rpy_data.x;
    rpytcmd.y = mpc_delay_rpy_data.y;
    rpytcmd.z = mpc_delay_rpy_data.z;
    /*rpytcmd.x = rpytcmd.y = 0;//Level
    rpytcmd.z = data.rpydata.z;//Set to current yaw
    */
    rpytcmd.w = (9.81/systemid.qrotor_gains(0));//Set to Default Value
    parserinstance->cmdrpythrust(rpytcmd, true);
    //double object_dist = roi_vel_ctrlr_->getObjectDistance();
    //ROS_INFO("Obj dist: %f",object_dist);
    //ROS_INFO("Sending zero rpy: %f,%f,%f, %f",rpytcmd.x, rpytcmd.y, rpytcmd.z, rpytcmd.w);
    /*{
      //Record Data
      QRotorSystemIDMeasurement measurement;
      measurement.t = (ros::Time::now() - mpc_request_time).toSec()-(2*delay_send_time_+0.02);//For we are using up 0.16 seconds for sending virtual controls to wakeup quadrotor

      measurement.position<<data.localpos.x, data.localpos.y, data.localpos.z;
      measurement.rpy<<data.rpydata.x, data.rpydata.y, data.rpydata.z;
      measurement.control.setZero();
      //measurement.control<<rpytcmd.w, current_u[1], current_u[2], current_u[3];
      //Record:
      systemid_measurements.push_back(measurement);
      control_measurements.push_back(Vector3d(rpytcmd.x, rpytcmd.y, rpytcmd.z));
    }
    */
    return;
  }
  //Check if thread can be joined [Done Optimizing]
  if(iterate_mpc_thread)
  {
    bool mpc_thread_iterating_copy_;
    mpc_thread_mutex.lock();
    mpc_thread_iterating_copy_ = mpc_thread_iterating;
    mpc_thread_mutex.unlock();
    if(mpc_thread_iterating_copy_)
    {
      ROS_WARN("Optimization not completed on time");
      mpctimer.stop();
      stateTransitionMPCControl(false);
      return;
    }
    else if(model_control.J() > 40)//Cost is high did not converge
    {
      //Publish Trajectory
      geometry_msgs::Vector3 localpos = data.localpos;
      //Publish position wrt to current data
      model_control.publishTrajectory(localpos, data.rpydata);
      ROS_WARN("Optimization did not succeed: %f",model_control.J());
      mpctimer.stop();
      stateTransitionMPCControl(false);
      return;
    }
  }

  if(!mpc_closed_loop_)
  {
    //ROS_INFO("Number of controls: %d, %d",model_control.us.size(), mpc_trajectory_count);
    //OPENLOOP VERSION:
    if(mpc_trajectory_count < model_control.us.size())
    {
      Eigen::Vector4d &current_u = model_control.us.at(mpc_trajectory_count);
      //gcop::QRotorIDState &current_x = model_control.xs.at(mpc_trajectory_count+1);
      rpytcmd.x = rpytcmd.x + model_control.stepSize()*current_u[1];
      rpytcmd.y = rpytcmd.y + model_control.stepSize()*current_u[2];
      //rpytcmd.w = (current_u[0]>85)?85:(current_u[0]<20)?20:current_u[0];//Simple Bounds so we dont send crazy values
      rpytcmd.w = (current_u[0]>100)?100:(current_u[0]<10)?10:current_u[0];//Simple Bounds so we dont send crazy values
      rpytcmd.z = rpytcmd.z + model_control.stepSize()*current_u[3];
      rpytcmd.z = rpytcmd.z > M_PI?(rpytcmd.z - 2*M_PI):(rpytcmd.z < -M_PI)?(rpytcmd.z + 2*M_PI):rpytcmd.z;//Make sure yaw command is reasonable
      mpc_trajectory_count++;
      parserinstance->cmdrpythrust(rpytcmd, true);
      //static ros::Time prev_time = ros::Time::now();
      //ROS_INFO("Current Control: %d, %f,%f,%f,%f",mpc_trajectory_count, rpytcmd.x, rpytcmd.y, rpytcmd.z, rpytcmd.w);
      //ROS_INFO("state.u[2]: %f,%f,%f",model_control.xs[0].u[2], model_control.xs[1].u[2], model_control.us[0][3]);
      //ROS_INFO("Current rate: %f,%f,%f",current_u[1], current_u[2], current_u[3]);
      //prev_time = ros::Time::now();

      {
        //Record Data
        QRotorSystemIDMeasurement measurement;
        measurement.t = (ros::Time::now() - mpc_request_time).toSec()-(2*delay_send_time_+0.02);//For we are using up 0.16 seconds for sending virtual controls to wakeup quadrotor

        measurement.position<<data.localpos.x, data.localpos.y, data.localpos.z;
        measurement.rpy<<data.rpydata.x, data.rpydata.y, data.rpydata.z;
        measurement.control<<rpytcmd.w, current_u[1], current_u[2], current_u[3];
        //Record:
        systemid_measurements.push_back(measurement);
        control_measurements.push_back(Vector3d(rpytcmd.x, rpytcmd.y, rpytcmd.z));
      }
    }
    else if((event.current_real - mpc_request_time).toSec() < model_control.tf() + 2*delay_send_time_+0.04)
    {
      //ROS_INFO("Sending constant rpy: %f,%f,%f, %f",rpytcmd.x, rpytcmd.y, rpytcmd.z, rpytcmd.w);
      parserinstance->cmdrpythrust(rpytcmd, true);//Send Last Command
      //Recording after trajectory is done but until delay_send_time
      {
        //Record Data
        QRotorSystemIDMeasurement measurement;
        measurement.t = (ros::Time::now() - mpc_request_time).toSec()-(2*delay_send_time_+0.02);

        measurement.position<<data.localpos.x, data.localpos.y, data.localpos.z;
        measurement.rpy<<data.rpydata.x, data.rpydata.y, data.rpydata.z;
        measurement.control<<rpytcmd.w, 0, 0, 0;
        //Record:
        systemid_measurements.push_back(measurement);
        control_measurements.push_back(Vector3d(rpytcmd.x, rpytcmd.y, rpytcmd.z));
      }
    }
    else
    {
      mpctimer.stop();
      stateTransitionMPCControl(false);

      //Publish Trajectory
      geometry_msgs::Vector3 localpos;
      localpos.x = systemid_measurements.begin()->position[0];
      localpos.y = systemid_measurements.begin()->position[1];
      localpos.z = systemid_measurements.begin()->position[2];
      geometry_msgs::Vector3 initial_rpy;
      initial_rpy.x = systemid_measurements.begin()->rpy[0];
      initial_rpy.y = systemid_measurements.begin()->rpy[1];
      initial_rpy.z = systemid_measurements.begin()->rpy[2];
      //Publish position wrt to current data
      model_control.publishTrajectory(localpos, initial_rpy);

      //Record Trajectory to a File
      logMeasurements(true);
    }
  }
  ///NOT IMPLEMENTED
  /*else
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
        model_control.iterate();//Iterate
        Eigen::Vector4d &current_u = model_control.us[0];//Get Current Control
        gcop::QRotorIDState &current_x = model_control.xs[1];

        rpytcmd.x = rpytcmd.x + model_control.step_size_*current_u[1];
        rpytcmd.y = rpytcmd.y + model_control.step_size_*current_u[2];
        rpytcmd.w = (current_u[0]>80)?80:(current_u[0]<20)?20:current_u[0];//Simple Bounds so we dont send crazy values
        rpytcmd.z = rpytcmd.z + model_control.step_size_*current_u[3];
        rpytcmd.z = rpytcmd.z > M_PI?(rpytcmd.z - 2*M_PI):(rpytcmd.z < -M_PI)?(rpytcmd.z + 2*M_PI):rpytcmd.z;//Make sure yaw command is reasonable

        rpytcmd.x = current_x.u[0];
        rpytcmd.y = current_x.u[1];
        rpytcmd.w = (current_u[0]>80)?80:(current_u[0]<20)?20:current_u[0];//Simple Bounds so we dont send crazy values
        rpytcmd.z = current_x.u[2];//TODO: Add Provision to send yaw cmd instead of yaw rate
        //If Needed do this. Iterate on a separate thread and send the closest state etc
        parserinstance->cmdrpythrust(rpytcmd, true);
      }
    }
    */
}
