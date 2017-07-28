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

#include <algorithm>
#include <functional>
#include <memory>

#include <nodelet/loader.h>
#include <ros/ros.h>
#include <royale_ros/Ls.h>
#include <royale_ros/Open.h>
#include <royale.hpp>

class CameraManager
{
public:
  CameraManager()
    : spinner_(new ros::AsyncSpinner(1))
  {
    ros::NodeHandle np("~");

    //---------------------
    // Advertised Services
    //---------------------
    this->ls_srv_ =
      np.advertiseService<royale_ros::Ls::Request,
                          royale_ros::Ls::Response>
      ("Ls", std::bind(&CameraManager::Ls, this,
                       std::placeholders::_1,
                       std::placeholders::_2));

    this->open_srv_ =
      np.advertiseService<royale_ros::Open::Request,
                          royale_ros::Open::Response>
      ("Open", std::bind(&CameraManager::Open, this,
                         std::placeholders::_1,
                         std::placeholders::_2));

  } // end: ctor

  /**
   * Run the event loop
   */
  void Run()
  {
    this->spinner_->start();
    ros::waitForShutdown();
  }

  /**
   * Implements the `Ls' service
   *
   * The `Ls' service is used to get a listing of available Royale cameras
   * connected to the system. The return value contains a list of strings. Each
   * string is a unique camera id that can be used for instantiating a specific
   * camera node for image acquisition.
   */
  bool Ls(royale_ros::Ls::Request &req,
          royale_ros::Ls::Response &res)
  {
    royale::CameraManager manager;
    auto camlist = manager.getConnectedCameraList();
    for (auto& id : camlist)
      {
        res.ids.push_back(std::string(id.c_str()));
      }
    return true;
  }

  /**
   * Implements the `Open' service
   *
   * Loads a camera nodlet based on the passed in `id' (the serial
   * number of the camera).
   */
  bool Open(royale_ros::Open::Request &req,
            royale_ros::Open::Response &res)
  {
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    this->loader_.load(req.id, "royale_ros/camera_nodelet", remap, nargv);
    return true;
  }

private:
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  nodelet::Loader loader_;

  ros::ServiceServer ls_srv_;
  ros::ServiceServer open_srv_;

}; // end: class CameraManager

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_manager");
  CameraManager().Run();
  return 0;
}
