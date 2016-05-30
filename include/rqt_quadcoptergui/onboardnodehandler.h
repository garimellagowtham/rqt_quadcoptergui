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

/**
 * @brief This class handles the commands from #rqt_quadcoptergui::QuadcopterGui and sends the commands to several subsystems. In that sense, this is the gui logic class which combines various subclasses and performs the gui commands.
 *
 * Subscribers:
 * - ctrltraj (gcop_comm/CtlrTraj): Trajectory to be tracked when (track_trajectory) is pressed
 * - goal(geometry_msgs/PoseStamped): receive goal pose for position control from rviz
 * - /gui_commands(rqt_quadcoptergui/GuiCommandMessage): Receive Gui commands which do not expect any return such as land arm disarm etc
 *
 * Publishers:
 * - /gui_state(rqt_quadcoptergui/GuiStateMessage): Send back transition of state coming from OnboardNodeHandler to Gui. For example automatic transitions
 * - /quad_status(std_msgs/String): Data from the quadrotor sent to Gui for display. This contains data from different subsystems along with quadcopter status (Landed in air etc)
 * - targetvel(visualization_msgs/Marker): Publish desired velocity when tracking object
 * - imu_rpy(geometry_msgs/Vector3): Euler angles coming from IMU for debug purposes
 * - global_vel(geometry_msgs/Vector3): Velocity in global frame
 *
 * Parameters:
 * - /control/obj_dist_max(double): The clipping distance for pixel depth in Roi Vel Controller
 * - /control/optimize_online(bool): Flag to decide whether to do parameter estimation online or not.
 * - /control/closedloop_estimation(bool): Flag to decide whether to do continous parameter estimation or only once. When rpyt mode is enabled and this flag is false, rpyt mode is switched off
 *    after measurement period. The optimization of parameters is performed only once in that way. If this flag is true, parameter estimation is performed continously even during velocity pos and
 * - mpc modes (They all command rpy in the end)
 * - /control/set_offsets_mpc(bool): If true, the body offsets from system id are transferred to mpc subsystems and velocity controller subsystems. If not, they are set to zeros
 * - /control/measurement_period(double): The time period until which the measurements and controls are collected before calling system ID
 * - /control/vel_send_time(double): When using virtual obstacle, this gives the time until which forward velocity is sent before switching to MPC mode.
 * - /control/virtual_obstacle(bool): If true, the enable mpc assumes there is a virtual obstacle. It sends forward velocity in the body frame for vel send time and then run MPC trajectory.
 * - /control/mpc_iterate_time(double): The upper bound on time taken to run mpc iteration until satisfactory solution is reached.
 *    This is used to delay the mpc trajectory execution until mpc iteration is completed
 * - /control/waypoint_mpc(bool): If true, expects waypoints when pressing enable mpc. The quadrotor automatically detects obstacles in between and avoids them as it moves towards the way points
 *    If false and virtual obstacle is also false, the quadrotor gives a forward velocity and waits until an obstacle is detected in range. Then it executes mpc trajectory. It does not track waypoints.
 *
 * - /arm/joint_speed(double): The default speed for arm in radians per second
 *
 * - /gui/uav_name(string): The frame id used to publish transforms to quadrotor. This frameid is also used to look up transforms to camera and arm
 * - /gui/logdir(string): The directory in which log folder is created. If not present, the logfiles will be saved in home directory.
 * - /gui/parser_plugin(string): The name of the parser to load. Currently supported pixhawk, ardrone, dji
 *
 *
 * Static Transformations:
 *  - uav_name to "camera": Transform from UAV frame to camera frame. The UAV is assumed to be NWU format at the geometric center.
 *  - uav_name to "arm" : Transform from UAV frame to base of arm.
 *
 * Timers:
 * - #armcmdtimer -> #armcmdTimerCallback(const ros::TimerEvent&)
 * - #mpcpostimer  -> #mpcpostimerCallback(const ros::TimerEvent&)
 * - #velcmdtimer -> #velcmdtimerCallback(const ros::TimerEvent&)
 * - #poscmdtimer -> #poscmdtimerCallback(const ros::TimerEvent&)
 * - #mpctimer -> #mpctimerCallback(const ros::TimerEvent&)
 * - #rpytimer -> #rpytimerCallback(const ros::TimerEvent&)
 * - #trajectorytimer -> #trajectorytimerCallback(const ros::TimerEvent&)
 * - #quadstatetimer -> #quadstatetimerCallback(const ros::TimerEvent&)
 * - #hometimer -> #poscmdtimerCallback(const ros::TimerEvent&)
 *
 * Subsystem Classes:
 * - #arm_model - Model of 2DOF arm to perform forward and inverse kinematics
 * - #arm_hardware_controller_ - Send joint angle and velocity commands, and gripper commands to an actual arm.
 * - #parser_loader - Plugin class to provide common interface among different quadrotors. The interface allows for sending velocity, rpy, position commands to quadrotors.
 *    In addition, a common data structure is introduced to get data from quadrotor. The getter function for quadrotor data is internally locked to avoid issues with simultaneous writing and reading issues
 * - #roi_vel_ctrlr_ - This is base class for roi velocity controller. It provides interface for getting position of the obstacle as well as velocity commands for going towards an object selected using roi.
 * - #vel_ctrlr_ - This class provides rpyt commands required to maintain a desired velocity. This is used to perform velocity control using rpyt mode, thereby avoiding switching modes in midflight.
 * - #pos_ctrlr_ - This class provides the velocity commands required to maintain a desired position. This is not the most stable way of performing positon control. But with appropriate gain tuning,
 *    the position controller provides the right velocity commands to acheive a desired position.
 * - #model_control - This class controls provides the mpc trajectory to avoid obstacles.
 * - #so3 - Provides functions for conversion on SO3 group. This is a just a reference member
 *
 *
 */

class OnboardNodeHandler
{
public:
    /**
     * @brief OnboardNodeHandler Constructor
     * @param nh_ Input NodeHandle
     */
    OnboardNodeHandler(ros::NodeHandle &nh_);

    ~OnboardNodeHandler();

protected:
    /** @name Subscribers
     */
    ///@{
    //ros::Subscriber camdata_sub;

    //ros::Subscriber joydata_sub;

    ros::Subscriber trajectory_subscriber_;///< Subscriber to gcop format trajectory for Visual servoing or any general trajectories

    ros::Subscriber gui_command_subscriber_;///< Commands from Gui to change any states

    ros::Subscriber goal_pose_subscriber_;///< Subscribe to waypoint command from rviz

    //ros::Subscriber guidance_obs_dist_;///< Get obstacle distance from Guidance
    ///@}

    /** @name Publishers
     */
    ///@{
    ros::Publisher gui_state_publisher_;///< Publish any changes in quadcopter state to update Gui

    ros::Publisher quad_state_publisher_;///< Publish quadrotor and other sensors state in terms of sensor data  etc

    //ros::Publisher jointstate_pub;

    //ros::Publisher armtarget_pub;

    ros::Publisher imu_rpy_pub_;///< Publish imu rpy for debug purposes if requested

    ros::Publisher global_vel_pub_;///< Publish linear velocity in global frame if requested

    ros::Publisher marker_pub_;///< Publish the initialized direction when tracking an object has been initialized
    ///@}
   

    /** @name Timers
     */
    ///@{
    ros::Timer armcmdtimer;///< @see #armcmdTimerCallback(const ros::TimerEvent&)
    ros::Timer mpcpostimer;///< @see #mpcpostimerCallback(const ros::TimerEvent&)
    ros::Timer velcmdtimer;///< @see #velcmdtimerCallback(const ros::TimerEvent&)
    ros::Timer poscmdtimer;///< @see #poscmdtimerCallback(const ros::TimerEvent&)
    ros::Timer mpctimer;///< @see #mpctimerCallback(const ros::TimerEvent&)
    ros::Timer rpytimer;///< @see #rpytimerCallback(const ros::TimerEvent&)
    //ros::Timer online_systemid_timer;//REFACTOR #TODO
    ros::Timer trajectorytimer;///< @see #trajectorytimerCallback(const ros::TimerEvent&)
    ros::Timer quadstatetimer;///< @see #quadstatetimerCallback(const ros::TimerEvent&)
    ros::Timer hometimer;///< @see #poscmdtimerCallback(const ros::TimerEvent&)
    ///@}


protected:
    //NodeHandle
    ros::NodeHandle &nh;///< Node Handle

    /** @name Subsystems
     * These systems actually incorporate all the MPC, obstacle avoidance , tracking logic etc
     */
    ///@{
    boost::shared_ptr<gcop::Arm> arm_model;///< Model of 2DOF arm to perform forward and inverse kinematics
    boost::shared_ptr<ArmHardwareController>  arm_hardware_controller_;///< Send joint angle and velocity commands, and gripper commands to an actual arm.
    //boost::shared_ptr<dynamixelsdk::DynamixelArm> arm_hardwareinst;
    boost::shared_ptr<pluginlib::ClassLoader<parsernode::Parser> > parser_loader;///< Load Parser plugin
    boost::shared_ptr<parsernode::Parser> parserinstance;///< Plugin class to provide common interface among different quadrotors. The interface allows for sending velocity, rpy, position commands to quadrotors. In addition, a common data structure is introduced to get data from quadrotor. The getter function for quadrotor data is internally locked to avoid issues with simultaneous writing and reading issues
    boost::shared_ptr<VelController> roi_vel_ctrlr_;///< This is base class for roi velocity controller. It provides interface for getting position of the obstacle as well as velocity commands for going towards an object selected using roi.
    boost::shared_ptr<QuadVelController> vel_ctrlr_;///< Controls velocity of quadrotor using rpyt
    boost::shared_ptr<QuadPosController> pos_ctrlr_;///< Controls position based on a path or a given desired pos
    QRotorIDModelControl model_control;///< MPC Controller for Quadrotor model
    SO3 &so3;///< Provides functions for conversion on SO3 group. This is a just a reference member
    ///@}

    // boost::shared_ptr<SetptCtrl> ctrlrinst;

    /** @name  Helper Variables
     */
    ///@{
    char buffer[1500];///< buffer for creating Text data for quadstate
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
    bool reset_parameters_systemid_;///< Used to reset system id parameters
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
    Eigen::Affine3d arm_cam_tf_eig_;///< Transform from arm base to camera
    double tolerance_tip_pos_;///< When the tip is closer than this the gripper will close around the object
    geometry_msgs::Vector3 object_position_mpc_start;///< Object Position at start of MPC
    geometry_msgs::Vector3 object_position_mpc_pred;///< Object Position predicted using delay time
    QRotorSystemIDMeasurement quad_meas_mpc_start;///< Object Position predicted using delay time
    //// Logger Variables:
    bool logdir_created;///< Indicates whether logdirectory has been created
    ofstream trajfile;
    //// TF:
    boost::shared_ptr<tf::TransformBroadcaster> broadcaster;///< Transform Broadcaster
    ///@}

    /** @name Threadlocked variables
     * Flags that require locking before accessing them
     */
    ///@{
    bool mpc_thread_iterate;///< Whether mpc is iterating
    bool systemid_flag_;///< Used to check if systemid is being performed or not
    bool fast_iterate_mpc;///< Whether to do a fast iteration or not
    ///@}

    /** @name Mutexes
     * control the access the access to threadlocked variables
     */
    ///@{
    boost::mutex mpc_thread_mutex;///Mutex to lock access to mpc thread iterate member  and fast iterate mpc
    boost::mutex systemid_thread_mutex;///< Member to lock access to system
    boost::recursive_mutex reconfig_mutex;///< Access to reconfigure variables. Reconfigure service has been moved to a separate thread. Hence access to any reconfigure variables should lock this mutex.
    parsernode::common::quaddata data;///< Quadcopter data from parser Function to get this member is internally locked by quadcopter parser
    ////@}

    /** @name Threads
     */
    ///@{
    boost::thread *iterate_systemid_thread;///< SystemID optimization thread @see #onlineOptimizeThread()
    boost::thread *reconfig_service_thread;///< Reconfigure Service thread @see #reconfigThread()
    boost::thread *iterate_mpc_thread;///< Iterate MPC thread @see #iterateMPCThread()
    ///@}

    //double waypoint_vel;///< Velocity with which to move to goal
    //double waypoint_yawvel;///< Velocity with which to move in yaw towards goal 

    /** @name  State Variables
     */
    ///@{
    bool enable_logging;///< If logging is enabled
    //bool enable_camctrl;///< If Camera control is enabled
    bool enable_rpytcontrol;///< If rpyt control is enabled
    bool enable_tracking;///< If we are tracking an object or not
    bool enable_velcontrol;///< If we enable quadcopter control
    bool enable_poscontrol;///< If we enable quadcopter position control
    bool enable_mpccontrol;///< If we enable quadcopter MPC Control
    bool enable_trajectory_tracking;///< If we are tracking a trajectory
    bool enable_arm;///< If arm should be active
    ///@}


    //// ROS Messages
    ////sensor_msgs::JointState jointstate_msg;///< For publishing arm state

    


    //ofstream camfile;
    //ofstream tipfile;
    //char camfile_buffer[FILE_BUFFER_SIZE];
		//char tipfile_buffer[FILE_BUFFER_SIZE];

    /** @name Parameters
     */
    ///@{
    bool publish_rpy;///< Publish roll pitch yaw on a topic or not
    //bool publish_vel;///< Publish velocity on a topic or not
    std::string uav_name;///< Name of UAV used in setting ID for tf
    string logdir;///< Log Directory Used by Logger
    string parserplugin_name;///< Name of the quadcopter Parser
    tf::StampedTransform CAM_QUAD_transform;///<transform from camera to Quadcopter
    tf::StampedTransform ARM_QUAD_transform;///<transform from arm to Quadcopter
    bool reconfig_init;///< Initialize reconfig with params
    bool reconfig_update;///< Update reconfig with vel
    //double goal_tolerance;///< Tolerance on when to stop closed loop MPC
    bool optimize_online_;///< Optimize the quadcopter parameters online
    bool set_offsets_mpc_;///< Set Offsets from online optimization to MPC
    double measurement_period;///< Measurement time for optimization
    double vel_send_time_;///< Velocity Send Time
    double delay_send_time_;///< Send dummy rpy for this time
    double mpc_iterate_time_;///< Time taken for mpc to iterate
    bool  virtual_obstacle_;///< If using virtual obstacle or not
    bool  waypoint_mpc_;///< Use mpc to follow waypoints and avoid obstacles
    //bool use_alvar_;///< Use alvar tracking instead of roi
    double arm_default_speed_;///< Default speed of the arm
    bool closedloop_estimation_;///< Estimate parameters in closed loop
    ///@}

protected:
    /** @name Helper Functions
     */
    ///@{

    /**
     * @brief get current state for MPC or system ID. This fills the state with latest quadrotor data
     * @param state output state
     */
    void getCurrentState(QRotorIDState &state);

    //void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);
    /**
     * @brief set the member variables to default variables. These cannot be set using member initializer lists usually.
     */
    inline void setupMemberVariables();
    /**
     * @brief load the ros parameter members
     */
    inline void loadParameters();
    /**
     * @brief create arm subsystem i.e both #arm_model and #arm_hardware_controller_ .
     * @todo Should return a bool if the creation of arm is successful or not
     */
    inline void createArmInstance();
    /**
     * @brief creates parser based on #parserplugin_name
     * @return true if quadparser is loaded and initialized successfully
     */
    inline bool createParserInstance();

    //inline bool createControllerInstance();
    /**
     * @brief creates a log Directory and sets the flag #logdir_created to true
     */
    inline void setupLogDir();
    /**
     * @brief logs the mpc or system id measurements to a file
     * @param mpc_flag If true logs mpc measurements otherwise logs systemid measurements
     */
    inline void logMeasurements(bool mpc_flag);
    /**
     * @brief helper function which loads the system id measurements and sets the #systemid_flag_ to true after the #measurement_period
     */
    inline void storeMeasurements();
    /**
     * @brief fills the initial state #systemid_init_state with the latest data from quadparser (Should not be called while system id is being performed)
     */
    inline void setInitialState();
    ///@}

    /** @name Gui State Transition Functions:
     * State transition logic functions. These functions are called by #receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg) . This can also be called by other members in the code which want to force in change of state
     */
    ///@{
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
    inline void clearVelController();
    ///@}

    /** @name Gui Button Command Functions:
     */
    ///@{
    inline void armQuad();
    inline void landQuad();
    inline void disarmQuad();
    inline void initializeMPC();
    ///@}

    /** @name ROS Subscriber Callbacks
     */
    ///@{
    //void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
    /**
     * @brief receive commands for state change Eg: #stateTransitionLogging(bool) or pure commands such as #landQuad() etc
     * @param command_msg
     */
    void receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg);
    /**
     * @brief receive Goal Pose from rviz for pos cmd
     * @param goal_pose input pose from rviz
     */
    void receiveGoalPose(const geometry_msgs::PoseStamped &goal_pose);

    //void receiveObstacleDistance(const sensor_msgs::LaserScan &scan);

    //void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);
    /**
     * @brief receive gcop trajectory to track. The #enable_trajectory_tracking flag needs to be set true for it to receive the trajectory. The trajectory is followed using a velocity timer
     * @param traj_msg The input trajectory.
     */
    void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);
    /**
     * @brief callback for reconfigure parameters
     * @param config input reconfig parameters
     * @param level level is used to segregate the parameters into groups
     */
    void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);
    ///@}

    /** @name Timer Callbacks
     */
    ///@{
    //void goaltimerCallback(const ros::TimerEvent&);
    /**
     * @brief When mpc is enabled, this timer decides what to do until actual mpc trajectory is started. If in waypoint mode, this timer tracks the way points. If in virtual obstacle mode,
     * this timer sends the forward velocity. If in real obstacle but non waypoint mode, it keeps track of obstacle position. This timer switches to mpc execution timer when the obstacle is within range

     */
    void mpcpostimerCallback(const ros::TimerEvent&);
    /**
     * @brief If tracking is enabled, uses roi vel controller to move towards the object. Otherwise, commands the velocity specified by reconfigure.
     */
    void velcmdtimerCallback(const ros::TimerEvent&);
    /**
     * @brief This timer is used to send position commands to quadrotor.
     * It either uses waypoint commands directly to send it to a recorded home position, or uses position controller sub class to send velocity commands to quadrotor.
     * The record_home button in reconfig is used to record the home position.
     */
    void poscmdtimerCallback(const ros::TimerEvent&);
    /**
     * @brief This timer sends the mpc trajectory controls to quadrotor in an openloop fashion. It checks if the mpc optimization has been successfully completed before sending the controls.
     * In waypoint mode, this restarts the mpcpostimer after the trajectory is executed to complete the waypoint tracking
     */
    void mpctimerCallback(const ros::TimerEvent&);
    /**
     * @brief This timer handles less priority tasks such as publishing imu rpy, global velocity, and sending the quadstatus to Gui.
     */
    void quadstatetimerCallback(const ros::TimerEvent&);
    /**
     * @brief This timer is used to send rpy commands in enable rpy mode. It maps the rcin commands from radio into rpy commands before sending them to the quadrotor.
     */
    void rpytimerCallback(const ros::TimerEvent&);
    /**
     * @brief This timer is used to track gcop trajectories from visual servoing or MPC using velocity controller. It receives the trajectory using ctrltraj topic.
     */
    void trajectorytimerCallback(const ros::TimerEvent&);
    /**
     * @brief If tracking is enabled and arm is enabled, moves the arm to grasp an object. Uses roi vel controller to find the pose of the object with respect to the arm
     */
    void armcmdTimerCallback(const ros::TimerEvent&);
    ///@}

    /** @name Thread Functions
     */
    ///@{
    /**
     * @brief thread to perform system id optimization when #systemid_flag_ is set to true
     */
    void onlineOptimizeThread();
    /**
     * @brief runs the reconfigure service at 50 Hz in a separate callback queue
     */
    void reconfigThread();
    /**
     * @brief perform MPC optimization for obstacle avoidance if #mpc_thread_iterate is set to true
     */
    void iterateMPCThread();
    ///@}
    //void closeAfterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object

    //void oneshotGrab(const ros::TimerEvent &); //Timer Callback for relaxing grip after grabbing

};

#endif // ONBOARDNODEHANDLER_H
