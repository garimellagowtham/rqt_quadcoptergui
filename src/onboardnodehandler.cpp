#include <rqt_quadcoptergui/onboardnodehandler.h>
//#define ARM_ENABLED

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh_):nh(nh_)
                                                            , broadcaster(new tf::TransformBroadcaster())
                                                            , logdir_created(false), enable_logging(false)
                                                            , joymsg_prevbutton(0), buttoncount(0)
                                                            , joymsg_prevbutton1(0), buttoncount1(0), enable_joy(true)
                                                            , followtraj(false), waitingfortrajectory(true), initialitrq(true), nearest_index_gcoptime(0)
                                                            , reconfiginit(false), updategoal_dynreconfig(false)
                                                            , armcmdrate(4), armratecount(0), gripped_already(false), newcamdata(false)
                                                            , enable_control(false), enable_integrator(false), enable_camctrl(false)
                                                            , tip_position(), goalcount(1), diff_goal()
{
  //initialize member variables
  ROS_INFO("Setting up Member Variables");
  setupMemberVariables();

  //Load Parameters:
  ROS_INFO("Loading Parameters");
  loadParameters();
 
  ROS_INFO("Creating Parser");
  if(!createParserInstance())
  {
    ROS_ERROR("Failed to create Quadcopter Parser");
    return;
  }

#ifdef ARM_ENABLED
  ROS_INFO("Creating Arm Hardware Instance");
  arm_hardwareinst.reset(new dynamixelsdk::DynamixelArm(dyn_deviceInd, dyn_baudnum));
#endif

  //Create Instances of Quadcopter Parser, Arm Controller, Quad Controller:
  ROS_INFO("Creating Quadcopter Controller");
  if(!createControllerInstance())
  {
    ROS_ERROR("Failed to create controller");
    return;
  }

  ROS_INFO("Creating GCOP Arm Instance");
  if(!createArmInstance())
  {
    ROS_ERROR("Failed to create Arm Inverse Kinematics controller");
    return;
  }
  ROS_INFO("Subscribing to Callbacks");
  //Subscribe to GuiCommands
  gui_command_subscriber_ = nh_.subscribe("/gui_commands", 10, &OnboardNodeHandler::receiveGuiCommands, this);
  //Subscribe to MOCAP:
  vrpndata_sub = nh_.subscribe(uav_posename,1,&OnboardNodeHandler::vrpnCallback,this);
  //Subscribe to Camera Estimator:
  camdata_sub = nh_.subscribe("/Pose_Est/objpose",1,&OnboardNodeHandler::camcmdCallback,this);
  //Subscribe to JoyStick:
  joydata_sub = nh_.subscribe("/joy",1,&OnboardNodeHandler::joyCallback,this);
  //Subscribe to GCOP Trajectory:
  gcoptraj_sub = nh_.subscribe("/mbsddp/traj_resp",1,&OnboardNodeHandler::gcoptrajectoryCallback,this);

  ROS_INFO("Advertising Topics");
  //Advertise Gui State:
  gui_state_publisher_ = nh_.advertise<rqt_quadcoptergui::GuiStateMessage>("/gui_state", 10);
  //Advertise Quad Parser Data:
  quad_state_publisher_ = nh_.advertise<std_msgs::String>("/quad_status", 10);
  //Advertise Iteration Request for Optimal Control
  iterationreq_pub = nh_.advertise<gcop_comm::Iteration_req>("/mbsddp/iteration_req",10);
  //Advertise Joint States of Manipulator
  jointstate_pub = nh_.advertise<sensor_msgs::JointState>("/movingrobot/joint_states",10);
  //Advertise Target position of Manipulator
  armtarget_pub = nh_.advertise<visualization_msgs::Marker>("armtarget", 10);

  //Connect to dynamic reconfigure server:
	ROS_INFO("Setting Up Reconfigure Sever");
  reconfigserver.reset(new dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>(nh_));
  reconfigcallbacktype = boost::bind(&OnboardNodeHandler::paramreqCallback, this, _1, _2);
  reconfigserver->setCallback(reconfigcallbacktype);

  //Create Timers:
  ROS_INFO("Creating Timers");
  //Timer to move goal dynamically
  goaltimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::goaltimerCallback,this);//50Hz So the goal can go upto 25 Hz  Nyquist rate
  goaltimer.stop();
  //Timer for grabbing object
  timer_grabbing = nh_.createTimer(ros::Duration(4), &OnboardNodeHandler::closeAfterGrabbing, this, true);//One shot timer
  timer_grabbing.stop();
  //Timer for relaxing gripper
  timer_relaxgrip = nh_.createTimer(ros::Duration(4), &OnboardNodeHandler::oneshotGrab, this, true);//One shot timer
  timer_relaxgrip.stop();
  //Timer for commanding quadcopter
  cmdtimer = nh_.createTimer(ros::Duration(0.02), &OnboardNodeHandler::cmdtimerCallback,this);//50Hz is the update rate of Quadcopter cmd
  cmdtimer.start();
  //Timer for sending quadcopter state
  quadstatetimer = nh_.createTimer(ros::Duration(0.05), &OnboardNodeHandler::quadstatetimerCallback, this);//20Hz update GUI quad state
}

OnboardNodeHandler::~OnboardNodeHandler()
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
  gui_command_subscriber_.shutdown();

  gui_state_publisher_.shutdown();
  quad_state_publisher_.shutdown();
  jointstate_pub.shutdown();
  armtarget_pub.shutdown();
  iterationreq_pub.shutdown();

  broadcaster.reset();

  reconfigserver.reset();

  goaltimer.stop();
  cmdtimer.stop();
  quadstatetimer.stop();

  vrpnfile.close();//Close the file
  camfile.close();//Close the file
  tipfile.close();//Close the file
}

//////////////////////HELPER Functions///////////////////
void OnboardNodeHandler::publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg)
{
  gui_state_publisher_.publish(state_msg);
}
inline void OnboardNodeHandler::setupMemberVariables()
{
  //object_markeroffset = tf::Vector3(0,0.08,-0.18);
  //object_markeroffset = tf::Vector3(0,0.05,-0.25);
  object_markeroffset = tf::Vector3(0,0.0,-0.2);//relative to the camera markers in Optitrack frame //For full camera control this SHOULD BE IN Object/Inertial Frame
  //-0.07 was prev guess

  quadoffset_object = tf::Vector3(0, -0.63, 0.05);//Where the quadcopter should stay relative to the markers This is manually adjusted based on the accuracy of the quadcopter

  arm_basewrtquad.setValue(0.0732, 0, -0.1);

	curr_goal = tf::Vector3(0,0,0);//Initial current goal

  //Set the Quadcopter in Optitrak frame
  UV_O.setIdentity();

  //Prepare joinstate msg:
  jointstate_msg.header.frame_id = "/movingrobot/baselink";
  jointstate_msg.position.resize(NOFJOINTS);
  jointstate_msg.name.push_back("airbasetolink1");
  jointstate_msg.name.push_back("link1tolink2");//Initialization

  // Prepare Iteration request:
  itrq.x0.statevector.resize(2*NOFJOINTS);
  itrq.xf.statevector.resize(2*NOFJOINTS);
  itrq.xf.basetwist.linear.x = 0; itrq.xf.basetwist.linear.y = 0; itrq.xf.basetwist.linear.z = 0;
  itrq.xf.basetwist.angular.x = 0; itrq.xf.basetwist.angular.y = 0; itrq.xf.basetwist.angular.z = 0;

  // Prepare Target cube pointer for visualizing object
  target_marker.id = 1;
  target_marker.ns = "targetpickup";
  target_marker.header.frame_id = "/optitrak";
  target_marker.action = visualization_msgs::Marker::ADD;
  target_marker.pose.orientation.w = 1.0;
  target_marker.type = visualization_msgs::Marker::CUBE;
  target_marker.scale.x = 0.04;
  target_marker.scale.y = 0.1;
  target_marker.scale.z = 0.1;
  target_marker.color.r = 1.0;
  target_marker.color.a = 1.0;
}

inline void OnboardNodeHandler::loadParameters()
{
  //VRPN Pose
  if(!nh.getParam("/gui/vrpn_pose",uav_posename))
  {
    uav_posename="/pixhawk/pose";//Default
    ROS_ERROR("Cannot load uav pose parameter");
  }

  if(!nh.getParam("/gui/parser_plugin",parserplugin_name))
  {
    parserplugin_name = "pixhawk";//Default
    ROS_ERROR("Cannot find parser_plugin parameter to load the parser");
  }

  nh.param<double>("/ctrlr/targetx",target[0],0);
  nh.param<double>("/ctrlr/targety",target[1],0);
  nh.param<double>("/ctrlr/targetz",target[2],0);
  nh.param<bool>("/ctrlr/partialcam_control",cam_partialcontrol, true);
  nh.param<bool>("/ctrlr/openloop_mode",openloop_mode,true);
  nh.param<std::string>("/gui/uav_name",uav_name,"pixhawk");
  nh.param<bool>("/gui/reset_imu",reset_imu,false);
  nh.param<bool>("/gui/test_ctrlr",testctrlr,true);
  nh.param<int>("/dynamixel/deviceIndex",dyn_deviceInd, 0);
  nh.param<int>("/dynamixel/baudrate",dyn_baudnum, 57600);
  nh.param<std::string>("/gui/logdir",logdir,"/home/gowtham");
  nh.param<double>("/vrpn/centerx",center_workspace[0], 0.67);
  nh.param<double>("/vrpn/centery",center_workspace[1], 0.9);
  nh.param<double>("/vrpn/centerz",center_workspace[2], 1.5);

#ifdef ARM_ENABLED
  nh.getParam("/ctrlr/timeout_grabbing", timeout_grabbing);
#else
  timeout_grabbing = 1.0;// Hardset timeout of 1 sec
#endif

  if(testctrlr)
    ROS_INFO("In Test Controller Mode");

  //Find Camera pose in Quad frame
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
}

inline bool OnboardNodeHandler::createArmInstance()
{
  arminst.reset(new gcop::Arm);
  arminst->l1 = 0.175;
  //arminst->l2 = 0.35;
  arminst->l2 = 0.42;
  arminst->x1 = 0.025;//Need to change this after measuring again TODO
  return true;
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

  //Set gripper state to neutral in the beginning:
  parserinstance->grip(0);

  parserinstance->getquaddata(data);
  //Wait till parser is initialized:
  ros::Time current_time = ros::Time::now();
  while((ros::Time::now() - current_time).toSec() < 15)
  {
    if(parserinstance->initialized)
      break;
    usleep(100000);//0.1 sec
  }
  return parserinstance->initialized;
}

inline bool OnboardNodeHandler::createControllerInstance()
{
  ctrlrinst.reset(new SetptCtrl(nh, broadcaster));
  {
    double xbias, ybias, rateyawbias;
    nh.getParam("/bias_vrpnx",xbias);
    nh.getParam("/bias_vrpny",ybias);
    nh.getParam("/bias_rateyaw",rateyawbias);
    ctrlrinst->setextbias(data.thrustbias, xbias, ybias, rateyawbias);
    ROS_INFO("X,Y, RateYaw Bias: %f\t%f\t%f",xbias, ybias, rateyawbias);
    //ctrlrinst->setextbias(data.thrustbias); //Fext initial guess comes from the parser. We will need to estimate it for some quadcopters if its used in commanding it.
  }
  return true;
}


inline void OnboardNodeHandler::setupLogDir()
{
  // Logger
  std::string logdir_append = logdir + "/session";
  std::string logdir_stamped = parsernode::common::addtimestring(logdir_append);
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
  if(ctrlrinst)
    ctrlrinst->setlogdir(logdir_stamped);
  if(parserinstance)
    parserinstance->setlogdir(logdir_stamped);

  logdir_created = true;//Specify that log directory has been created
}

////////////////////////STATE TRANSITIONS/////////////////////////
inline void OnboardNodeHandler::stateTransitionController(bool state)
{
  //Set the extbias back to nominal value
  ctrlrinst->setextbias(data.thrustbias); //Set the external force back to nominal value
  if(state == true)
  {
    stateTransitionIntegrator(true);
    stateTransitionLogging(true);
    enable_control = true;
    goaltimer.start();
  }
  else
  {
    enable_control = false;
    stateTransitionIntegrator(false);
    goaltimer.stop();
  }
  //Publish Change of State:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_control;
  state_message.commponent_id = state_message.controller_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionCameraController(bool state)
{
  //Testing only Optitrack
  if(!ctrlrinst)
  {
    ROS_WARN("Controller not instantiated");
		goto PUBLISH_CAMERA_STATE;
  }

  if(state == true)
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
    }//else full camera control
  }
  else
  {
    cout<<"Disabling_Camctrl"<<endl;
    /////////////////////////This cannot be used as a fallback without optitrack system //////////////
    enable_camctrl = false;
    followtraj = false;
    waitingfortrajectory = true;//Resetting trajectory wait after one trial in open/closedloop mode
    initialitrq = true;//Reset initialization after one trial
    stateTransitionTrajectoryTracking(false);//Transition the trajectory tracking to false
  }
  //Publish Change of State:
PUBLISH_CAMERA_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_camctrl;
  state_message.commponent_id = state_message.camera_controlstatus;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionTrajectoryTracking(bool state)
{
  if(state == true)
  {
    followtraj = true;
    //[DEBUG]
    ROS_INFO("Following traj true");
  }
  else if(state == false)
  {
    followtraj = false;

    center_workspace[2]  = curr_goal[2];//Set same height
    updategoal_dynreconfig = true;//Set the flag to make sure dynamic reconfigure reads the new goal
    goalcount = 20; //Set the goal back to the specified posn smoothly
    diff_goal.setValue((-curr_goal[0] + center_workspace[0])/goalcount, (-curr_goal[1] + center_workspace[1])/goalcount,(-curr_goal[2] + center_workspace[2])/goalcount);
    goaltimer.start();//Redundancy
  }
  //Publish Change of State:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = followtraj;
  state_message.commponent_id = state_message.trajectory_tracking_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionLogging(bool state)
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot Log");
		goto PUBLISH_LOGGING_STATE;
  }
  if(!ctrlrinst)
  {
    ROS_WARN("Controller Instance not defined. Cannot Log");
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
  //Publish Change of State:
PUBLISH_LOGGING_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_logging;
  state_message.commponent_id = state_message.log_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionJoyControl(bool state)
{
  //Get ros NodeHandle from parent nodelet manager:
  joymsg_prevbutton = 0; buttoncount = 0;
  joymsg_prevbutton1 = 0; buttoncount1 = 0;
  if(state == true)
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
  }
#endif
  //Publish Change of State:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_joy;
  state_message.commponent_id = state_message.joystick_status;
  gui_state_publisher_.publish(state_message);
}

inline void OnboardNodeHandler::stateTransitionIntegrator(bool state)
{
  if(!ctrlrinst)
  {
    ROS_WARN("Ctrl inst not defined");
		enable_integrator = false;
		goto PUBLISH_INTEGRATOR_STATE;
  }
  if(testctrlr)//If testing the controller, do not integrate the thrust
  {
		enable_integrator = false;
    ctrlrinst->integrate(false);//Redundancy
		goto PUBLISH_INTEGRATOR_STATE;
  }
  if(state == true)
  {
		enable_integrator = true;
    ctrlrinst->integrate(true);
  }
  else if(state == false)
  {
		enable_integrator = false;
    ctrlrinst->integrate(false);
  }
  //Publish Change of State:
PUBLISH_INTEGRATOR_STATE:
  rqt_quadcoptergui::GuiStateMessage state_message;
  state_message.status = enable_integrator;
  state_message.commponent_id = state_message.integrator_status;
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
  //Set the extbias back to nominal value
  ctrlrinst->setextbias(data.thrustbias); //Set the external force back to nominal value Just extra safety its already set in check_control etc
  parserinstance->takeoff();
}

inline void OnboardNodeHandler::disarmQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  rescmdmsg.x = 0;rescmdmsg.y = 0; rescmdmsg.z = 0; rescmdmsg.w = 0;//reset command
  parserinstance->disarm();
  //Uncheck the controller checkbox to be consistent:
  stateTransitionController(false);
}

inline void OnboardNodeHandler::landQuad()
{
  if(!parserinstance)
  {
    ROS_WARN("Parser Instance not defined. Cannot take off");
    return;
  }
  parserinstance->land();
  rescmdmsg.x = 0;rescmdmsg.y = 0; rescmdmsg.z = 0; rescmdmsg.w = 0;//reset command
  //Uncheck the controller checkbox to be consistent:
  stateTransitionController(false);
}

////////////////////////CALLBACKS//////////////////////
void OnboardNodeHandler::vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)
{
  //Check if vrpn is skipping:
  transformStampedMsgToTF(*currframe,UV_O);//converts to the right format  and stores the message
  ctrlrinst->Update(UV_O);//Updates the internal state

  tf::Matrix3x3 rotmat = UV_O.getBasis();
  rotmat.getEulerYPR(vrpnrpy[2],vrpnrpy[1],vrpnrpy[0]);
  if(enable_logging)
  {
    //Logging save to file
    vrpnfile<<(UV_O.stamp_.toNSec())<<"\t"<<(currframe->transform.translation.x)<<"\t"<<(currframe->transform.translation.y)<<"\t"<<(currframe->transform.translation.z)<<"\t"<<(currframe->transform.rotation.x)<<"\t"<<(currframe->transform.rotation.y)<<"\t"<<(currframe->transform.rotation.z)<<"\t"<<(currframe->transform.rotation.w)<<"\t"<<0<<"\t"<<0<<"\t"<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<endl;
  }

#ifndef LOG_DEBUG
  if(!data.armed)//Once the quadcopter is armed we do not set the goal position to quad's origin, the user will set the goal. But the goal will not move until u set the enable_control The user should not give random goal once it is initialized.
  {
    curr_goal = UV_O.getOrigin();//set the current goal to be same as the quadcopter origin we dont care abt the orientation as of now
    ctrlrinst->setgoal(curr_goal[0],curr_goal[1],curr_goal[2],goalyaw);//Set the goal to be same as the current position of the quadcopter the velgoal is by default 0
  }
#endif

  //Set the altitude of the quadcopter in the data
  parserinstance->setaltitude(currframe->transform.translation.z);
}

void OnboardNodeHandler::receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg)
{
  //Do whatever is commanded
  switch(command_msg.commponent_name)
  {
  case command_msg.enable_log://0
    stateTransitionLogging(command_msg.command);
    break;
  case command_msg.enable_integrator://1
    stateTransitionIntegrator(command_msg.command);
    break;
  case command_msg.traj_on://2
    stateTransitionTrajectoryTracking(command_msg.command);
    break;
  case command_msg.enable_ctrl://3
    stateTransitionController(command_msg.command);
    break;
  case command_msg.enable_joy://4
    stateTransitionJoyControl(command_msg.command);
    break;
  case command_msg.enable_cam://5
    stateTransitionCameraController(command_msg.command);
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

void OnboardNodeHandler::camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe)
{
  //Find the object pose in Quadcopter frame:
  tf::Transform OBJ_CAM_transform;
  tf::transformMsgToTF(currframe->transform,OBJ_CAM_transform);//converts to the right format
  tf::Transform OBJ_QUAD_transform = CAM_QUAD_transform*OBJ_CAM_transform*OBJ_MOD_transform;
  //+ quatRotate(UV_O.getRotation(),manual_offset) //Will incorporate it later
  OBJ_QUAD_stamptransform  = tf::StampedTransform(OBJ_QUAD_transform, currframe->header.stamp,uav_name,"object");//CAM_QUAD is the pose of Camera in Quadcopter frame, OBJ_MOD is the transform needed to make the object parallel (in terms of roll, pitch) with the Inertial frame

  if(!enable_camctrl)//Not Using Camera Control
  {
    return;
  }

  //Using Camera Control:
  if(!ctrlrinst)
  {
    ROS_WARN("CamCmdCallback:[Controller not instantiated]");
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

    tf::Vector3 ARMTarget_OPTITRACK_position = object_origin + object_markeroffset;

    target_marker.pose.position.x = ARMTarget_OPTITRACK_position[0];//Target Object Position
    target_marker.pose.position.y = ARMTarget_OPTITRACK_position[1];
    target_marker.pose.position.z = ARMTarget_OPTITRACK_position[2];
    armtarget_pub.publish(target_marker);

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

        //Closed loop mode needs itrq.basepose.twist to be found #TODO

        for(int count1 = 0;count1 < 2*NOFJOINTS; count1++) //Joint angles and vel
          itrq.x0.statevector[count1] = actual_armstate[count1];

        //Goal Condition:
        itrq.xf.basepose.translation.x = 0.65;
        itrq.xf.basepose.translation.y = 1.3;
        //itrq.xf.basepose.translation.y = 1.15;
        itrq.xf.basepose.translation.z =  1.91;

        tf::Quaternion finalorientation = tf::createQuaternionFromYaw(M_PI/2);//Final yaw  Ideally this should be obtained from object pose TODO
        quaternionTFToMsg(finalorientation, itrq.xf.basepose.rotation);
        //Find final posn angles based on offset quad posn:
        tf::Vector3 offsetquadlocaltarget = (object_markeroffset) - quadoffset_object;
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
  }//else full famera control Not implemented

  if(enable_logging)
  {
    geometry_msgs::TransformStamped objmsg;
    transformStampedTFToMsg(OBJ_QUAD_stamptransform,objmsg);//converts to the right format

    //Logging save to file
    camfile<<(OBJ_QUAD_stamptransform.stamp_.toNSec())<<"\t"<<(objmsg.transform.translation.x)<<"\t"<<(objmsg.transform.translation.y)<<"\t"<<(objmsg.transform.translation.z)<<"\t"<<(objmsg.transform.rotation.x)<<"\t"<<(objmsg.transform.rotation.y)<<"\t"<<(objmsg.transform.rotation.z)<<"\t"<<(objmsg.transform.rotation.w)<<"\t"<<curr_goal[0]<<"\t"<<curr_goal[1]<<"\t"<<curr_goal[2]<<"\t"<<object_origin[0]<<"\t"<<object_origin[1]<<"\t"<<object_origin[2]<<"\t"<<(UV_O.stamp_.toNSec())<<endl;
  }
}

void OnboardNodeHandler::joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg)
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

void OnboardNodeHandler::gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg)
{
  if(!followtraj)//If follow trajectory has not been enabled do not receive a trajectory
    return;
  //[DEBUG]
  ROS_INFO("Received Gcop Trajectory");
  gcop_trajectory = traj_msg;//Default Copy constructor (Instead if needed can write our own copy constructor)
  waitingfortrajectory = false;
  nearest_index_gcoptime = 0;//Every time we get a new trajectory we reset the time index for searching nearest current time to zero
  if(openloop_mode)//In Closed loop we usually have fewer number of iterations vs in openloop where we do lots of iterations. Thus by the time we receive the traj we have some offset which we remove
  {
    request_time = ros::Time::now();
  }
}

void OnboardNodeHandler::paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config, uint32_t level)
{
  // Use the config values to set the goals and gains for quadcopter
  if(!parserinstance || !ctrlrinst)
  {
    ROS_WARN("Parser or ctrlr not defined");
    return;
  }

  if(!reconfiginit)
  {
		ROS_INFO("Initializing Reconfig Params");
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

  ctrlrinst->setgains(config.kpr, config.kdr, config.kpt, config.kdt, config.kit);//Need to add kpy and kiy TODO
  ctrlrinst->setbounds(config.throtbound, (M_PI/180.0)*config.rpbound);//This throttle bound is different from throttle bias which needs to be estimated for some quadcopters. This just cuts off the throttle values that are beyond thrustbias +/- throtbound

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
      //[DEBUG]if(enable_control)
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
}

void OnboardNodeHandler::goaltimerCallback(const ros::TimerEvent &event)
{

  //Check how delayed cmdtimer is :
  if((event.current_real - event.last_real).toSec() > 0.06)
  {
    ROS_WARN("GoalTimer not catching up");
    ROS_INFO("Current Expected: %f\t%f",event.current_expected.toSec(), event.current_real.toSec());
  }

#ifdef ARM_ENABLED
  double armres = -1e3;//Initialize arm result
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
    target_location = (OBJ_QUAD_stamptransform.getOrigin() + quatRotate(UV_O.getRotation().inverse(),object_markeroffset)) - arm_basewrtquad;

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
        if(arminst && parserinstance && arm_hardwareinst) //Can also add enable_control flag for starting this only when controller has started TODO
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
              stateTransitionCameraController(false);
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
        stateTransitionCameraController(false);
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

void OnboardNodeHandler::cmdtimerCallback(const ros::TimerEvent &event)
{
  /////Add a clause for doing this when camcallback is not running///////
  //Call the ctrlr to set the ctrl and then send the command to the quadparser
  if(!ctrlrinst)
  {
    ROS_WARN("Controller not instantiated");
    return;
  }

  //Check how delayed cmdtimer is :
  if((event.current_real - event.last_real).toSec() > 0.04)
  {
    ROS_WARN("CmdTimer not catching up");
    ROS_INFO("Current Expected: %f\t%f",event.current_expected.toSec(), event.current_real.toSec());
  }

  controllers::ctrl_command rescmd;

  if(enable_camctrl && !cam_partialcontrol)//This implies we are doing full camera control
  {
    //Not Using motion capture But still need UV_O in partial cam ctrl so always useful to save the currframe when available if not available thats fine
    return;
  }

  //Store the current position of the quadcopter for display
  //Using kalman filter
  ctrlrinst->Getctrl(rescmd);//Since imu is corrected using unbiased vrpn data we can send commands which are unbiased too

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
  if(data.armed && enable_control)
  {
    parserinstance->cmdrpythrust(rescmdmsg,true);//Also controlling yaw
  }
}

void OnboardNodeHandler::quadstatetimerCallback(const ros::TimerEvent &event)
{
  if(!parserinstance)
  {
    //ROS_ERROR("No parser instance created");
    return;
  }

  parserinstance->getquaddata(data);

  //Reset attitude on IMU every 20 Hz
  {
    static int count_imu = 0;
    static tf::Vector3 imu_vrpndiff;
    if(count_imu < 10)
    {
      imu_vrpndiff = (1.0/double(count_imu+1))*(double(count_imu)*imu_vrpndiff + tf::Vector3((vrpnrpy[0] - data.rpydata.x),(vrpnrpy[1] - data.rpydata.y),(vrpnrpy[2] - data.rpydata.z)));
      count_imu++;
      ROS_INFO("Imu Offset: %f,%f,%f\t IMU: %f,%f,%f\t VRPNRPY: %f,%f,%f", imu_vrpndiff[0], imu_vrpndiff[1], imu_vrpndiff[2], data.rpydata.x, data.rpydata.y, data.rpydata.z, vrpnrpy[0], vrpnrpy[1], vrpnrpy[2]);
    }
    else
    {
      if(parserinstance)
      {
        if(reset_imu)
          parserinstance->reset_attitude(vrpnrpy[0]-imu_vrpndiff[0], vrpnrpy[1]-imu_vrpndiff[1], vrpnrpy[2]-imu_vrpndiff[2]);
      }
    }
  }
  tf::Vector3 quadorigin = UV_O.getOrigin();
  tf::Vector3 obj_origin = OBJ_QUAD_stamptransform.getOrigin();
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
          ,0.0, 0.0, 0.0
          ,(rescmdmsg.x)*(180/M_PI), (rescmdmsg.y)*(180/M_PI), rescmdmsg.z, rescmdmsg.w
          ,0.0, 0.0, 0.0
          ,obj_origin[0], obj_origin[1], obj_origin[2]
          ,tip_position[0], tip_position[1], tip_position[2]
          ,data.mass,data.timestamp,data.quadstate.c_str());

  //Publish State:
  std_msgs::String string_msg;
  string_msg.data = std::string(buffer);
  quad_state_publisher_.publish(string_msg);
}


void OnboardNodeHandler::closeAfterGrabbing(const ros::TimerEvent &event)
{
  ROS_INFO("Closing the arm and grabbing target");

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
  stateTransitionCameraController(false);
}

void OnboardNodeHandler::oneshotGrab(const ros::TimerEvent &event)
{
  parserinstance->grip(0);//Relax the grip
  ROS_INFO("Relaxing Grip");
}
