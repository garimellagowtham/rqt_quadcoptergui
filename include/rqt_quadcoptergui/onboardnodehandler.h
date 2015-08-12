#ifndef ONBOARDNODEHANDLER_H
#define ONBOARDNODEHANDLER_H

#include <ros/ros.h>

// Linux functions for creating directories etc
#include <sys/types.h>
#include <sys/stat.h>

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
#include <visualization_msgs/Marker.h>
#include <gcop_comm/CtrlTraj.h>
#include <gcop_comm/Iteration_req.h>
#include <rqt_quadcoptergui/GuiCommandMessage.h>
#include <rqt_quadcoptergui/GuiStateMessage.h>


//TF#include <sys/types.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define NOFJOINTS 2 //Arm joints

class OnboardNodeHandler
{
public:
    OnboardNodeHandler(ros::NodeHandle &nh_);

    virtual ~OnboardNodeHandler();

protected:
    ////Subscribers
    ros::Subscriber vrpndata_sub;

    ros::Subscriber camdata_sub;

    ros::Subscriber joydata_sub;

    ros::Subscriber gcoptraj_sub;

    ros::Subscriber gui_command_subscriber_;

    /////Publishers
    ros::Publisher gui_state_publisher_;

    ros::Publisher quad_state_publisher_;

    ros::Publisher iterationreq_pub;

    ros::Publisher jointstate_pub;

    ros::Publisher armtarget_pub;

    ros::Publisher imu_rpy_pub_;

    ros::Publisher vrpn_rpy_pub_;

    //// TF:
    boost::shared_ptr<tf::TransformBroadcaster> broadcaster;//Transform Broadcaster

    ////Timers
    ros::Timer goaltimer;//REFACTOR
    ros::Timer cmdtimer;//REFACTOR #TODO
    ros::Timer quadstatetimer;// Send quad state to GUI

    /////Reconfigure Server
    boost::shared_ptr<dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig> >reconfigserver;
		dynamic_reconfigure::Server<rqt_quadcoptergui::QuadcopterInterfaceConfig>::CallbackType reconfigcallbacktype;

protected:
    //NodeHandle
    ros::NodeHandle &nh;

    ///SubClasses of OnboardNodeHandler:
    boost::shared_ptr<gcop::Arm> arminst;
    boost::shared_ptr<dynamixelsdk::DynamixelArm> arm_hardwareinst;
    boost::shared_ptr<pluginlib::ClassLoader<parsernode::Parser> > parser_loader;
    boost::shared_ptr<parsernode::Parser> parserinstance;
    boost::shared_ptr<SetptCtrl> ctrlrinst;

    ///// Helper Variables
    int goalcount;///< Goal count used to propagate the goal
    bool waitingfortrajectory;///< Used to not send iteration requests when optimizer is already working on one
    bool initialitrq;///< Specifies whether this is the first iteration request for the quadcopter (For Openloop mode)
    bool newcamdata;///< Used to tell the arm if new cam data has arrived
    int nearest_index_gcoptime;///< Current nearest index on gcop trajectory in terms of time
    bool reconfiginit;///< Initialize reconfigure with parameters
    bool updategoal_dynreconfig;///< Flag for updating the dynamic reconfigure goal parameters whenever it is triggered (This will write the values in dynreconfig instead of reading from it)
    int armcmdrate;///< Factor by which arm command rate slows down
    int armratecount;///< Variable used to count upto armcmdrate
    bool gripped_already; ///< Used to avoid restarting the timer if it already gripped the target once just waits for 5 secs before returning
    ros::Time start_grabbing;///< Time when we start grabbing the object
    char buffer[1500];//buffer for creating Text data
    ros::Timer timer_grabbing;//Used to fold arm and set gripper to neutral after grabbing
    ros::Timer timer_relaxgrip;//Used to fold arm and set gripper to neutral after grabbing
		int count_imu;//Used to count the IMU VRPN Diff. Used by arming function to reset the count back to zero to freshly calculate new imu vrpn difference
		tf::Vector3 imu_vrpndiff;//Difference between IMU and VRPN


    //// State Variables
    bool enable_logging;///< If logging is enabled
    bool followtraj;///< Whether we are following a trajectory or doing position control
    bool enable_camctrl;///< If Camera control is enabled
    bool enable_control;///< Tells whether the controller is enabled or not
    bool enable_integrator;///< Tells whether integrator is enabled or not
    bool enable_joy;///< If Joystick mode is enabled

    //// ROS Messages
    sensor_msgs::JointState jointstate_msg;///< For publishing arm state
    gcop_comm::Iteration_req itrq;//Request for iteration
    gcop_comm::CtrlTraj gcop_trajectory;
    visualization_msgs::Marker target_marker;//For visualizing the object to grab

    //// Tf Variables:
    tf::StampedTransform UV_O;///< Storing the current frame along with time stamp:
    tf::StampedTransform OBJ_QUAD_stamptransform;///< Storing the current object pose in Quadcopter frame
    tf::Vector3  object_markeroffset;///< Offset of object wrt to camera markers
    tf::Vector3 quadoffset_object;///< Offset of the quadcopter from camera markers
    tf::Vector3 arm_basewrtquad;///< Offset of arm wrt to quadcopter origin (Optitrack)
    tf::Vector3 vrpnrpy;///< Latest vrpnrpy from vrpnCallback

    /////Ctrlr Variables
    tf::Vector3 curr_goal;///< Current goal to quadcopter
    tf::Vector3 diff_goal;///< Difference goal to propagate in goal timer
    float goalyaw;///< Current goal yaw for the position controller
    ros::Time request_time;///< Time when Optimal control request sent
    geometry_msgs::Quaternion rescmdmsg;///< Resulting controller command being sent

    /////Arm Variables:
    double as[2][3];//Arm inverse kinematics output
    double armlocaltarget[3];//Arm goal (Where the object is to grab)
    double actual_armstate[2*NOFJOINTS];//The angles obtained from dynamixelsdk
    double cmd_armstate[2*NOFJOINTS];//The angles of the arm in radians in gcop convention and angular velocities
    double tip_position[3];//Tip Position

    ////Joystick Variables
    int joymsg_prevbutton, buttoncount;
    int joymsg_prevbutton1, buttoncount1;


    //// Parser Variables
    parsernode::common::quaddata data;///< Quadcopter data from parser

    //// Logger Variables:
    bool logdir_created;///< Indicates whether logdirectory has been created
    ofstream vrpnfile;
    ofstream camfile;
    ofstream tipfile;

    //// Parameters:
    std::string uav_posename;
    tf::Vector3 target;//Extraction target point
    bool cam_partialcontrol;//Only use the object position to set the goal position
    bool publish_rpy;///< Publish roll pitch yaw on a topic or not
    double timeout_grabbing;//Timeout for waiting to grab object usually a very short time to just stay for few seconds
    bool openloop_mode;///< Optimal Control being used in open loop or closed loop
    std::string uav_name;///< Name of UAV used in setting ID for tf
    bool reset_imu;///< Reset the imu using EKF
    bool testctrlr;///< Sets the controller in test mode with very little throttle
    int dyn_deviceInd;///< Dynamixel Device Index (ttyUSB*)
    int dyn_baudnum;///< Dynamixel Baudrate (57600, 115200, etc)
    string logdir;///< Log Directory Used by Logger
    string parserplugin_name;///< Name of the quadcopter Parser
    tf::StampedTransform CAM_QUAD_transform;
    tf::StampedTransform OBJ_MOD_transform;
    tf::Vector3 center_workspace;

protected:
    // Helper Functions
    void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);

    inline void setupMemberVariables();

    inline void loadParameters();

    inline bool createArmInstance();

    inline bool createParserInstance();

    inline bool createControllerInstance();

    inline void setupLogDir();

    //Gui State Transition Functions:
    inline void stateTransitionController(bool);
    inline void stateTransitionCameraController(bool);
    inline void stateTransitionTrajectoryTracking(bool);
    inline void stateTransitionLogging(bool);
    inline void stateTransitionJoyControl(bool);
    inline void stateTransitionIntegrator(bool);

    //Gui Button Command Functions:
    inline void armQuad();
    inline void landQuad();
    inline void disarmQuad();

    //ROS Callbacks
    void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);

    void receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg);

    void camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);

    void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);

    void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

    void goaltimerCallback(const ros::TimerEvent&);

    void cmdtimerCallback(const ros::TimerEvent&);

    void quadstatetimerCallback(const ros::TimerEvent&);

    void closeAfterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object

    void oneshotGrab(const ros::TimerEvent &); //Timer Callback for relaxing grip after grabbing

};

#endif // ONBOARDNODEHANDLER_H
