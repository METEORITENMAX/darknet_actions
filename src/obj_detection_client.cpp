#include <string>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <darknet_actions/obj_detectionAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <naoqi_wrapper_msgs/NaoQi_animatedSayAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_detection_client");
  actionlib::SimpleActionClient<darknet_actions::obj_detectionAction> ac_objdetection("objectdetection", true);
  actionlib::SimpleActionClient<naoqi_wrapper_msgs::NaoQi_animatedSayAction> ac_animated("/naoqi_animatedSay_server/animatedSay",true);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_objdetection.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  darknet_actions::obj_detectionGoal goal;
  naoqi_wrapper_msgs::NaoQi_animatedSayGoal goalsay;
  goal.to_detected_obj = "cup";
  ac_objdetection.sendGoal(goal);
  //wait for the action to return
  bool finished_before_timeout = ac_objdetection.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac_objdetection.getState();
    ROS_INFO_STREAM(ac_objdetection.getResult()->obj_pos);
    //goalsay = ac_objdetection.getResult()->obj_pos.toString().c_str();
    ac_animated.waitForServer();
    std::string result_data = std::to_string(ac_objdetection.getResult()->obj_pos);
    goalsay.animatedMessage.data = "das objekt befindet sich an position "+result_data;
    ac_animated.sendGoal(goalsay);
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
