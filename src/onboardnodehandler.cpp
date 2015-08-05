#include <rqt_quadcoptergui/onboardnodehandler.h>

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh_)
{
  //Subscribe to GuiCommands
  gui_command_subscriber_ = nh_.subscribe("/gui_commands", 10, &OnboardNodeHandler::receiveGuiCommands, this);

  //Advertise Gui State:
  gui_state_publisher_ = nh_.advertise<rqt_quadcoptergui::GuiStateMessage>("/gui_state", 10);
}

void OnboardNodeHandler::receiveGuiCommands(const rqt_quadcoptergui::GuiCommandMessage &command_msg)
{
  //Do whatever is commanded
  switch(command_msg.commponent_name)
  {
  case command_msg.arm_quad :
    ROS_INFO("Arming Quad");
    break;
  }
}

void OnboardNodeHandler::publishGuiState(const rqt_quadcoptergui::GuiStateMessage &state_msg)
{
  gui_state_publisher_.publish(state_msg);
}
