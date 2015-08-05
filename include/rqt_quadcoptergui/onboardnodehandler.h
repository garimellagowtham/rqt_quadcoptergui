#ifndef ONBOARDNODEHANDLER_H
#define ONBOARDNODEHANDLER_H

#include <ros/ros.h>

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


//TF
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define NOFJOINTS 2 //Arm joints

class OnboardNodeHandler
{
public:
    OnboardNodeHandler(ros::NodeHandle &nh_);

    void receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg);

protected:
    ros::Subscriber gui_command_subscriber_;
    ros::Publisher gui_state_publisher_;

protected:
    void publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg);
};

#endif // ONBOARDNODEHANDLER_H
