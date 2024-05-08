/*
Copyright (c) 2024, Marco Faroni
Politecnico di Milano marco.faroni@polimi.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ur_dashboard_msgs/Load.h>
#include <ur_msgs/SetSpeedSliderFraction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "startup_node");

  std::string external_control_urp = "ros1.urp";
  std::string hw_namespace = "ur_hardware_interface";
  double speed_slider_value=1.0;

  ros::NodeHandle nh;
  ros::ServiceClient client_load_program = nh.serviceClient<ur_dashboard_msgs::Load>("/"+hw_namespace+"/dashboard/load_program");
  ros::ServiceClient client_start_program = nh.serviceClient<std_srvs::Trigger>("/"+hw_namespace+"/dashboard/play");
  ros::ServiceClient client_stop_program =  nh.serviceClient<std_srvs::Trigger>("/"+hw_namespace+"/dashboard/stop");
  ros::ServiceClient client_speed_slider_on_startup =  nh.serviceClient<ur_msgs::SetSpeedSliderFraction>("/"+hw_namespace+"/set_speed_slider");

  ros::NodeHandle pnh("~");
  if (!pnh.getParam("speed_slider_value",speed_slider_value))
  {
    ROS_WARN("speed_slider_value not set. Deafult = %d", speed_slider_value);
  }


  if (!client_load_program.waitForExistence(ros::Duration(10.0)))
  {
    ROS_ERROR("Timeout: load program");
  }
  if (!client_start_program.waitForExistence(ros::Duration(10.0)))
  {
    ROS_ERROR("Timeout: start program");
  }
  if (!client_stop_program.waitForExistence(ros::Duration(10.0)))
  {
    ROS_ERROR("Timeout: stop program");
  }
  if (!client_speed_slider_on_startup.waitForExistence(ros::Duration(10.0)))
  {
    ROS_ERROR("Timeout: set_speed_slider");
  }

  ROS_INFO_STREAM(hw_namespace << ": stopping current program (if any)");
  std_srvs::Trigger req_stop;
  client_stop_program.call(req_stop);
  if (!req_stop.response.success)
  {
    ROS_ERROR("Failed to stop current program");
    ROS_ERROR_STREAM("msg: " << req_stop.response.message);
  }
  ros::WallDuration(1.0).sleep();

  ROS_INFO_STREAM(hw_namespace << ": loading program " << external_control_urp );
  ur_dashboard_msgs::Load req_load;
  req_load.request.filename = external_control_urp;
  client_load_program.call(req_load);
  if (!req_load.response.success)
  {
    ROS_ERROR_STREAM("Failed to load program" << external_control_urp);
    ROS_ERROR_STREAM("msg: " << req_load.response.answer);
  }
  ros::WallDuration(1.0).sleep();

  ROS_INFO_STREAM(hw_namespace << ": starting loaded program.");
  std_srvs::Trigger req_start;
  client_start_program.call(req_start);
  if (!req_start.response.success)
  {
    ROS_ERROR_STREAM("Failed to start program " << external_control_urp);
    ROS_ERROR_STREAM("msg: " << req_start.response.message);
  }
  ros::WallDuration(1.0).sleep();

  ROS_INFO_STREAM(hw_namespace << ": setting speed slider to initial value " << speed_slider_value);
  ur_msgs::SetSpeedSliderFraction set_slider_req;
  set_slider_req.request.speed_slider_fraction = speed_slider_value;
  client_speed_slider_on_startup.call(set_slider_req);
  if (!set_slider_req.response.success)
  {
    ROS_ERROR_STREAM("Failed to set speed slider");
  }


  ROS_INFO_STREAM("Startup node ok");

  return 0;

}
