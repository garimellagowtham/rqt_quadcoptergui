#ifndef ONBOARDNODEHANDLER_H
#define ONBOARDNODEHANDLER_H

#include <ros/ros.h>

// Linux functions for creating directories etc
#include <sys/types.h>
#include <sys/stat.h>

//Roi Vel Include:
#include <controllers/roivelcontroller.h>

//Arm Controller
#include <dynamixelsdk/arm_helper.h>
#include <controllers/SetptCtrl.h>
#include <controllers/arm.h>

//Dynamic Reconfigure
#include <rqt_quadcoptergui/QuadcopterInterfaceConfig.h>
#include <dynamic_reconfigure/server.h>

//Quadcopter Parsers
#include <parsernode/parser.h>
#include <pluginlib/class_loader.h>

//Boost Threads
#include <boost/thread/mutex.hpp>

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

//Gcop Quad Model MPC Controller
#include <gcop_ctrl/qrotoridmodelcontrol.h>

//Gcop Quad Model Identifier:
#include <gcop/qrotorsystemid.h>

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

    virtual ~OnboardNodeHandler();

protected:
    ////Subscribers
    //ros::Subscriber camdata_sub;

    //ros::Subscriber joydata_sub;

    ros::Subscriber trajectory_subscriber_;///< Trajectory to be tracked

    ros::Subscriber gui_command_subscriber_;

    ros::Subscriber goal_pose_subscriber_;///< Subscribe to waypoint command from rviz

    /////Publishers
    ros::Publisher gui_state_publisher_;

    ros::Publisher quad_state_publisher_;

    //ros::Publisher jointstate_pub;

    //ros::Publisher armtarget_pub;

    ros::Publisher imu_rpy_pub_;

    ros::Publisher marker_pub_;


    //// TF:
    boost::shared_ptr<tf::TransformBroadcaster> broadcaster;//Transform Broadcaster

    ////Timers
    ros::Timer velcmdtimer;//REFACTOR #TODO
    ros::Timer poscmdtimer;//REFACTOR #TODO
    ros::Timer mpctimer;//REFACTOR #TODO
    ros::Timer rpytimer;//REFACTOR #TODO
    ros::Timer rpy_stop_timer;//REFACTOR #TODO
    ros::Timer trajectorytimer;//REFACTOR #TODO
    ros::Timer quadstatetimer;// Send quad state to GUI

    /////Reconfigure Server
    boost::shared_ptr<dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig> >reconfigserver;
		dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>::CallbackType reconfigcallbacktype;

protected:
    //NodeHandle
    ros::NodeHandle &nh;

    ///SubClasses of OnboardNodeHandler:
    //boost::shared_ptr<gcop::Arm> arminst;
    //boost::shared_ptr<dynamixelsdk::DynamixelArm> arm_hardwareinst;
    boost::shared_ptr<pluginlib::ClassLoader<parsernode::Parser> > parser_loader;
    boost::shared_ptr<parsernode::Parser> parserinstance;
    boost::shared_ptr<RoiVelController> roi_vel_ctrlr_;
    QRotorIDModelControl model_control;///< MPC Controller for Quadrotor model
    QRotorSystemID systemid;///< System Identification class from GCOP
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
    ros::Time rpytimer_start_time;///< When rpytimer started
    string logdir_stamped_;///< Name of Log Directory
    Vector3d initial_state_vel_;///< MPC Initial State Velocity
    Vector4d home_pose_;///< Home Position and Yaw before starting rpytimer OR Wherever it is recorded
    //////////////SYSTEM ID HELPER VARIABLES////////////////////
    vector<QRotorSystemIDMeasurement> systemid_measurements;///< System ID Measurements
    vector<Vector3d> control_measurements;///< Control Measurements [ONLY FOR LOGGING]
    //QRotorIDState systemid_init_state;///< Initial State for System Identification
    double prev_rp_cmd[2];///< Previous roll and pitch commands for finding control rate
    double prev_ctrl_time;///< Previous control time for roll and pitch commands in System ID
    
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


    //// ROS Messages
    ////sensor_msgs::JointState jointstate_msg;///< For publishing arm state

    /////Arm Variables:
    /*double as[2][3];//Arm inverse kinematics output
    double armlocaltarget[3];//Arm goal (Where the object is to grab)
    double actual_armstate[2*NOFJOINTS];//The angles obtained from dynamixelsdk
    double cmd_armstate[2*NOFJOINTS];//The angles of the arm in radians in gcop convention and angular velocities
    double tip_position[3];//Tip Position
    */

    //// Parser Variables
    parsernode::common::quaddata data;///< Quadcopter data from parser

    //// Logger Variables:
    bool logdir_created;///< Indicates whether logdirectory has been created
    //ofstream camfile;
    //ofstream tipfile;
    //char camfile_buffer[FILE_BUFFER_SIZE];
		//char tipfile_buffer[FILE_BUFFER_SIZE];

    //// Parameters:
    bool publish_rpy;///< Publish roll pitch yaw on a topic or not
    std::string uav_name;///< Name of UAV used in setting ID for tf
    string logdir;///< Log Directory Used by Logger
    string parserplugin_name;///< Name of the quadcopter Parser
    tf::StampedTransform CAM_QUAD_transform;///<transform from camera to Quadcopter
    bool reconfig_init;///< Initialize reconfig with params
    bool reconfig_update;///< Update reconfig with vel
    double goal_tolerance;///< Tolerance on when to stop closed loop MPC
    bool optimize_online_;///< Optimize the quadcopter parameters online
    bool set_offsets_mpc_;///< Set Offsets from online optimization to MPC
    double measurement_period;///< Measurement time for optimization
    double vel_send_time_;///< Velocity Send Time
    double rpy_dummy_send_time_;///< Send dummy rpy for this time
    //bool test_vel;///< Test velocity by sending velocity in the beginning of rpytimer Code

protected:
    // Helper Functions
    void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);

    inline void setupMemberVariables();

    inline void loadParameters();

    //inline bool createArmInstance();

    inline bool createParserInstance();

    //inline bool createControllerInstance();

    inline void setupLogDir();

    inline void logMeasurements(bool mpc_flag);

    //inline void setInitialStateMPC();

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
    //inline void stateTransitionJoyControl(bool);
    //inline void stateTransitionIntegrator(bool);

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

    //void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);

    void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);

    void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

    //void goaltimerCallback(const ros::TimerEvent&);

    void velcmdtimerCallback(const ros::TimerEvent&);

    void poscmdtimerCallback(const ros::TimerEvent&);

    void mpctimerCallback(const ros::TimerEvent&);

    void quadstatetimerCallback(const ros::TimerEvent&);

    void rpytimerCallback(const ros::TimerEvent&);

    void onlineOptimizeCallback(const ros::TimerEvent&);

    void trajectorytimerCallback(const ros::TimerEvent&);

    //void closeAfterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object

    //void oneshotGrab(const ros::TimerEvent &); //Timer Callback for relaxing grip after grabbing

};

#endif // ONBOARDNODEHANDLER_H
