#ifndef ONBOARDNODEHANDLER_H
#define ONBOARDNODEHANDLER_H

#include <ros/ros.h>

// Linux functions for creating directories etc
#include <sys/types.h>
#include <sys/stat.h>

//Roi Vel Include:
#include <controllers/roivelcontroller.h>
#include <controllers/alvartrackcontroller.h>

//Arm Controller
#include <dynamixelsdk/arm_helper.h>
#include <controllers/SetptCtrl.h>
#include <controllers/quadvelcontroller.h>
#include <controllers/quadposcontroller.h>

#include <controllers/arm.h>
#include <controllers/arm_hardware_controller.h>

//Dynamic Reconfigure
#include <rqt_quadcoptergui/QuadcopterInterfaceConfig.h>
#include <dynamic_reconfigure/server.h>

//Quadcopter Parsers
#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

//Boost Threads
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

//Ros Messages:
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <rqt_quadcoptergui/GuiCommandMessage.h>
#include <rqt_quadcoptergui/GuiStateMessage.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>

//Gcop Quad Model MPC Controller
#include <gcop_ctrl/qrotoridmodelcontrol.h>

//Gcop Quad Model Identifier:
#include <gcop/qrotorsystemid.h>

//Load systemid Parameters
#include <rqt_quadcoptergui/load_systemid_parameters.h>

//TF#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define NOFJOINTS 2 //Arm joints

#ifndef FILE_BUFFER_SIZE
#define FILE_BUFFER_SIZE 1024
#endif

class OnboardNodeHandler
{
public:
    OnboardNodeHandler(ros::NodeHandle &nh_);

    ~OnboardNodeHandler();

protected:
    ////Subscribers
    //ros::Subscriber camdata_sub;

    //ros::Subscriber joydata_sub;

    ros::Subscriber trajectory_subscriber_;///< Trajectory to be tracked

    ros::Subscriber gui_command_subscriber_;

    ros::Subscriber goal_pose_subscriber_;///< Subscribe to waypoint command from rviz

    ros::Subscriber guidance_obs_dist_;///< Get obstacle distance from Guidance

    /////Publishers
    ros::Publisher gui_state_publisher_;

    ros::Publisher quad_state_publisher_;

    //ros::Publisher jointstate_pub;

    //ros::Publisher armtarget_pub;

    ros::Publisher imu_rpy_pub_;
    ros::Publisher global_vel_pub_;

    ros::Publisher marker_pub_;
   

    //// TF:
    boost::shared_ptr<tf::TransformBroadcaster> broadcaster;//Transform Broadcaster

    ////Timers
    ros::Timer armcmdtimer;
    ros::Timer mpcpostimer;//REFACTOR #TODO
    ros::Timer velcmdtimer;//REFACTOR #TODO
    ros::Timer poscmdtimer;//REFACTOR #TODO
    ros::Timer mpctimer;//REFACTOR #TODO
    ros::Timer rpytimer;//REFACTOR #TODO
    //ros::Timer online_systemid_timer;//REFACTOR #TODO
    ros::Timer trajectorytimer;//REFACTOR #TODO
    ros::Timer quadstatetimer;// Send quad state to GUI
    ros::Timer hometimer;///< Go to Home


protected:
    //NodeHandle
    ros::NodeHandle &nh;

    ///SubClasses of OnboardNodeHandler:
    boost::shared_ptr<gcop::Arm> arm_model;
    boost::shared_ptr<ArmHardwareController>  arm_hardware_controller_;
    //boost::shared_ptr<dynamixelsdk::DynamixelArm> arm_hardwareinst;
    boost::shared_ptr<pluginlib::ClassLoader<parsernode::Parser> > parser_loader;
    boost::shared_ptr<parsernode::Parser> parserinstance;
    boost::shared_ptr<VelController> roi_vel_ctrlr_;
    boost::shared_ptr<QuadVelController> vel_ctrlr_;///< Controls velocity of quadrotor using rpyt
    boost::shared_ptr<QuadPosController> pos_ctrlr_;///< Controls position based on a path or a given desired pos
    QRotorIDModelControl model_control;///< MPC Controller for Quadrotor model
    SO3 &so3;

    // boost::shared_ptr<SetptCtrl> ctrlrinst;

    ///// Helper Variables
    char buffer[1500];//buffer for creating Text data
    geometry_msgs::Vector3 desired_vel;///< Commanded velocity to quadcopter
    geometry_msgs::Quaternion rpytcmd;///< Commanded rpyt msg
    geometry_msgs::Vector3 goal_position;///< Goal position for waypoint control
    double desired_yaw;///< Commanded Yaw from feedforward
    double goal_altitude;///< For Position Control goal altitude
    int Nit;///< Number of iterations to run for MPC
    int mpc_trajectory_count;///< Count on mpc trajectory 
    bool mpc_closed_loop_;///< Internal state to switch between openloop and closed loop modes of MPC
    gcop_comm::CtrlTraj gcop_trajectory;///< Trajectory received for tracking
    int nearest_index_gcop_trajectory;///< Nearest index for gcop trajectory
    ros::Time gcop_trajectory_request_time;///< Time when gcop trajectory is requested
    ros::Time mpc_request_time;///< Time when mpc is enabled
    double kp_trajectory_tracking;///< Trajectory tracking gain on velocity
    double timeout_trajectory_tracking;///< Timeout on when to stop trajectory tracking in sec
    double timeout_mpc_control;///< Timeout on mpc closed loop control
    //ros::Time rpytimer_start_time;///< When rpytimer started
    string logdir_stamped_;///< Name of Log Directory
    //Vector3d initial_state_vel_;///< MPC Initial State Velocity
    ros::Time home_start_time;//When go home command is pressed
    bool systemid_flag_;///< Used to check if systemid is being performed or not
    ros::Time systemid_complete_time_;///< Time when systemid is completed
    //////////////SYSTEM ID HELPER VARIABLES////////////////////
    vector<QRotorSystemIDMeasurement> systemid_measurements;///< System ID Measurements
    vector<Vector3d> control_measurements;///< Control Measurements [ONLY FOR LOGGING]
    QRotorIDState vel_ctrlr_state;///< State for velocity controller
    //double prev_rp_cmd[2];///< Previous roll and pitch commands for finding control rate
    QRotorIDState systemid_init_state;///< Initial State for System Identification
    double prev_ctrl_time;///< Previous control time for roll and pitch commands in System ID
    int meas_filled_;///< Number of measurements filled
    geometry_msgs::Vector3 mpc_delay_rpy_data;///< RPY Commands sent during start of 

    /// Thread for Iterating:
    bool mpc_thread_iterate;///< Whether mpc is iterating
    bool fast_iterate_mpc;///< Whether to do a fast iteration or not
    boost::mutex mpc_thread_mutex;
    boost::mutex systemid_thread_mutex;
    boost::recursive_mutex reconfig_mutex;

    /// Thread for system ID
    boost::thread *iterate_systemid_thread;
    boost::thread *reconfig_service_thread;
    boost::thread *iterate_mpc_thread;///< Iterate MPC

    //double waypoint_vel;///< Velocity with which to move to goal
    //double waypoint_yawvel;///< Velocity with which to move in yaw towards goal 

    //// State Variables
    bool enable_logging;///< If logging is enabled
    //bool enable_camctrl;///< If Camera control is enabled
    bool enable_rpytcontrol;///< If rpyt control is enabled
    bool enable_tracking;///< If we are tracking an object or not
    bool enable_velcontrol;///< If we enable quadcopter control
    bool enable_poscontrol;///< If we enable quadcopter position control
    bool enable_mpccontrol;///< If we enable quadcopter MPC Control
    bool enable_trajectory_tracking;///< If we are tracking a trajectory
    bool enable_arm;///< If arm should be active


    //// ROS Messages
    ////sensor_msgs::JointState jointstate_msg;///< For publishing arm state

    /////Arm Variables:
    Eigen::Affine3d arm_cam_tf_eig_;///< Transform from arm base to camera
    double tolerance_tip_pos_;
    //double as[2][3];//Arm inverse kinematics output
    //double armlocaltarget[3];//Arm goal (Where the object is to grab)
    //double actual_armstate[2*NOFJOINTS];//The angles obtained from dynamixelsdk
    //double cmd_armstate[2*NOFJOINTS];//The angles of the arm in radians in gcop convention and angular velocities
    //double tip_position[3];//Tip Position
    geometry_msgs::Vector3 object_position_mpc_start;///< Object Position at start of MPC
    geometry_msgs::Vector3 object_position_mpc_pred;///< Object Position predicted using delay time
    QRotorSystemIDMeasurement quad_meas_mpc_start;///< Object Position predicted using delay time
    

    //// Parser Variables
    parsernode::common::quaddata data;///< Quadcopter data from parser

    //// Logger Variables:
    bool logdir_created;///< Indicates whether logdirectory has been created
    ofstream trajfile;
    //ofstream camfile;
    //ofstream tipfile;
    //char camfile_buffer[FILE_BUFFER_SIZE];
		//char tipfile_buffer[FILE_BUFFER_SIZE];

    //// Parameters:
    bool publish_rpy;///< Publish roll pitch yaw on a topic or not
    bool publish_vel;///< Publish velocity on a topic or not
    std::string uav_name;///< Name of UAV used in setting ID for tf
    string logdir;///< Log Directory Used by Logger
    string parserplugin_name;///< Name of the quadcopter Parser
    tf::StampedTransform CAM_QUAD_transform;///<transform from camera to Quadcopter
    tf::StampedTransform ARM_QUAD_transform;///<transform from arm to Quadcopter
    bool reconfig_init;///< Initialize reconfig with params
    bool reconfig_update;///< Update reconfig with vel
    double goal_tolerance;///< Tolerance on when to stop closed loop MPC
    bool optimize_online_;///< Optimize the quadcopter parameters online
    bool set_offsets_mpc_;///< Set Offsets from online optimization to MPC
    double measurement_period;///< Measurement time for optimization
    double vel_send_time_;///< Velocity Send Time
    double delay_send_time_;///< Send dummy rpy for this time
    double mpc_iterate_time_;///< Time taken for mpc to iterate
    bool  virtual_obstacle_;///< If using virtual obstacle or not
    bool  waypoint_mpc_;///< Use mpc to follow waypoints and avoid obstacles
    bool use_alvar_;///< Use alvar tracking instead of roi
    double arm_default_speed_;///< Default speed of the arm
    bool closedloop_estimation_;///< Estimate parameters in closed loop

protected:
    // Helper Functions
    void getCurrentState(QRotorIDState &state);
    void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);

    inline void setupMemberVariables();

    inline void loadParameters();

    inline void createArmInstance();

    inline bool createParserInstance();

    //inline bool createControllerInstance();

    inline void setupLogDir();

    inline void logMeasurements(bool mpc_flag);

    inline void storeMeasurements();

    inline void setInitialState();

    //Gui State Transition Functions:
    //inline void stateTransitionController(bool);
    //inline void stateTransitionManualTargetRetrieval(bool);
    //inline void stateTransitionCameraController(bool);
    //inline void stateTransitionTrajectoryTracking(bool);
    inline void stateTransitionLogging(bool);
    inline void stateTransitionTracking(bool);
    inline void stateTransitionVelControl(bool);
    inline void stateTransitionPosControl(bool);
    inline void stateTransitionMPCControl(bool);
    inline void stateTransitionTrajectoryTracking(bool);
    inline void stateTransitionRpytControl(bool);
    inline void stateTransitionEnableArm(bool);
    //inline void stateTransitionJoyControl(bool);
    //inline void stateTransitionIntegrator(bool);
    inline void clearSystemID();

    //Gui Button Command Functions:
    inline void armQuad();
    inline void landQuad();
    inline void disarmQuad();
    inline void initializeMPC();

    //ROS Callbacks
    //void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);

    void receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg);

    void receiveCameraInfo(const sensor_msgs::CameraInfo &info);

    void receiveRoi(const sensor_msgs::RegionOfInterest &roi_rect);

    void receiveGoalPose(const geometry_msgs::PoseStamped &goal_pose);

    //void receiveObstacleDistance(const sensor_msgs::LaserScan &scan);

    //void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);

    void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);

    void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

    //void goaltimerCallback(const ros::TimerEvent&);

    void mpcpostimerCallback(const ros::TimerEvent&);

    void velcmdtimerCallback(const ros::TimerEvent&);

    void poscmdtimerCallback(const ros::TimerEvent&);

    void mpctimerCallback(const ros::TimerEvent&);

    void quadstatetimerCallback(const ros::TimerEvent&);

    void rpytimerCallback(const ros::TimerEvent&);

    void onlineOptimizeThread();

    void reconfigThread();

    void iterateMPCThread();

    void trajectorytimerCallback(const ros::TimerEvent&);
  
    void armcmdTimerCallback(const ros::TimerEvent& event);

    //void closeAfterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object

    //void oneshotGrab(const ros::TimerEvent &); //Timer Callback for relaxing grip after grabbing

};

#endif // ONBOARDNODEHANDLER_H
