#ifndef ONBOARDNODEHANDLER_H
#define ONBOARDNODEHANDLER_H

#include <ros/ros.h>

// Linux functions for creating directories etc
#include <sys/types.h>
#include <sys/stat.h>

//Roi Vel Include:
#include <rqt_quadcoptergui/roi_to_vel.h>

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

    ros::Subscriber gui_command_subscriber_;

    ros::Subscriber roi_subscriber_;///< Subscribe to the roi of object

    ros::Subscriber camera_info_subscriber_;///< Subscribe to camera info

    /////Publishers
    ros::Publisher gui_state_publisher_;

    ros::Publisher quad_state_publisher_;

    //ros::Publisher jointstate_pub;

    //ros::Publisher armtarget_pub;

    ros::Publisher imu_rpy_pub_;

    ros::Publisher vel_marker_pub_;


    //// TF:
    boost::shared_ptr<tf::TransformBroadcaster> broadcaster;//Transform Broadcaster

    ////Timers
    //ros::Timer goaltimer;//REFACTOR
    //ros::Timer cmdtimer;//REFACTOR #TODO
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
    // boost::shared_ptr<SetptCtrl> ctrlrinst;

    ///// Helper Variables
    char buffer[1500];//buffer for creating Text data
    geometry_msgs::Vector3 desired_vel_track;///< Tracking velocity
    double desired_yaw_rate;///< Tracking yaw rate

    //// State Variables
    bool enable_logging;///< If logging is enabled
    //bool enable_camctrl;///< If Camera control is enabled
    bool enable_tracking;///< If we are tracking an object or not


    //// ROS Messages
    ////sensor_msgs::JointState jointstate_msg;///< For publishing arm state
    visualization_msgs::Marker vel_marker;//For visualizing the object to grab
    boost::shared_ptr<sensor_msgs::CameraInfo> intrinsics;

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
    ofstream vrpnfile;
    ofstream camfile;
    ofstream tipfile;
		char vrpnfile_buffer[FILE_BUFFER_SIZE];
		char camfile_buffer[FILE_BUFFER_SIZE];
		char tipfile_buffer[FILE_BUFFER_SIZE];

    //// Parameters:
    bool publish_rpy;///< Publish roll pitch yaw on a topic or not
    std::string uav_name;///< Name of UAV used in setting ID for tf
    double yaw_gain;///< Gain on Yaw vel for tracking object
    double vel_mag;///< Magnitude of vel for tracking object
    string logdir;///< Log Directory Used by Logger
    string parserplugin_name;///< Name of the quadcopter Parser
    tf::StampedTransform CAM_QUAD_transform;///<transform from camera to Quadcopter
    bool reconfig_init;///< Initialize reconfig with params

protected:
    // Helper Functions
    void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);

    inline void setupMemberVariables();

    inline void loadParameters();

    //inline bool createArmInstance();

    inline bool createParserInstance();

    //inline bool createControllerInstance();

    inline void setupLogDir();

    //Gui State Transition Functions:
    //inline void stateTransitionController(bool);
    //inline void stateTransitionManualTargetRetrieval(bool);
    //inline void stateTransitionCameraController(bool);
    //inline void stateTransitionTrajectoryTracking(bool);
    inline void stateTransitionLogging(bool);
    inline void stateTransitionTracking(bool);
    //inline void stateTransitionJoyControl(bool);
    //inline void stateTransitionIntegrator(bool);

    //Gui Button Command Functions:
    inline void armQuad();
    inline void landQuad();
    inline void disarmQuad();

    //ROS Callbacks
    //void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);

    void receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg);

    void receiveCameraInfo(const sensor_msgs::CameraInfo &info);

    void receiveRoi(const sensor_msgs::RegionOfInterest &roi_rect);

    //void camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);

    //void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);

    //void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);

    void paramreqCallback(rqt_quadcoptergui::QuadcopterInterfaceConfig &config , uint32_t level);

    //void goaltimerCallback(const ros::TimerEvent&);

    //void cmdtimerCallback(const ros::TimerEvent&);

    void quadstatetimerCallback(const ros::TimerEvent&);

    //void closeAfterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object

    //void oneshotGrab(const ros::TimerEvent &); //Timer Callback for relaxing grip after grabbing

};

#endif // ONBOARDNODEHANDLER_H
