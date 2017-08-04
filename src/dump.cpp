/*
 * Copyright (C) 2017 Love Park Robotics, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distribted on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <iostream>
#include <string>
#include <ros/ros.h>
#include "royale_ros/Dump.h"

int main(int argc, char **argv)
{
  std::string service_name;

  ros::init(argc, argv, "royale_ros_dump");

  ros::NodeHandle nh("~");
  nh.param("srv", service_name, std::string("Dump"));

  ros::ServiceClient client =
    nh.serviceClient<royale_ros::Dump>(service_name);

  royale_ros::Dump srv;
  if (client.call(srv))
    {
      std::cout << srv.response.config << std::endl;
    }
  else
    {
      ROS_ERROR("Failed to call `Dump' service!");
      return 1;
    }

  return 0;
}
