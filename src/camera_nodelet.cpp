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

#include <royale_ros/camera_nodelet.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <royale_ros/ExposureTimes.h>
#include <royale_ros/GetUseCases.h>
#include <royale.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

void
royale_ros::CameraNodelet::onInit()
{
  NODELET_INFO_STREAM("onInit(): " << this->getName());

  // Currently, we only support LEVEL 1 access to the device.
  this->access_level_ = 0;

  // flag indicating that we have not yet created our image publishers
  this->instantiated_publishers_ = false;

  // used for detecting a disconnected camera
  this->last_frame_ = ros::Time::now();

  this->np_ = getMTPrivateNodeHandle();
  this->it_.reset(new image_transport::ImageTransport(this->np_));
  this->np_.param<std::string>("serial_number", this->serial_number_, "-");
  this->np_.param<float>("poll_bus_secs", this->poll_bus_secs_, 1.);
  this->np_.param<float>("timeout_secs", this->timeout_secs_, 1.);

  //------------------------------------------------------------
  // Instantiate the underlying camera device by polling the bus
  //------------------------------------------------------------
  this->timer_ =
    this->np_.createTimer(ros::Duration(.001),
                          [this](const ros::TimerEvent& t)
                          { this->InitCamera(); },
                          true); // oneshot timer

  //---------------------
  // Advertised Services
  //---------------------
  this->get_use_cases_srv_ =
    this->np_.advertiseService<royale_ros::GetUseCases::Request,
                               royale_ros::GetUseCases::Response>
    ("GetUseCases", std::bind(&CameraNodelet::GetUseCases, this,
                              std::placeholders::_1,
                              std::placeholders::_2));
}

void
royale_ros::CameraNodelet::InitCamera()
{
  std::lock_guard<std::mutex> lock(this->cam_mutex_);

  // For an already initialized camera, this acts as a heartbeat
  if (this->cam_ != nullptr)
    {
      {
        // New scope to check to see if the camera has timedout
        std::lock_guard<std::mutex> lock(this->last_frame_mutex_);
        if ((ros::Time::now() - this->last_frame_).toSec() >
            this->timeout_secs_)
          {
            NODELET_WARN_STREAM("Camera timeout!");
            this->cam_.reset();
          }
      }

      this->RescheduleTimer();
      return;
    }


  NODELET_INFO_STREAM("Probing for available royale cameras...");
  royale::CameraManager manager;
  auto camlist = manager.getConnectedCameraList();

  if (! camlist.empty())
    {
      if (this->serial_number_ == "-")
        {
          // grab the first camera found
          this->cam_ = manager.createCamera(camlist.at(0));
          this->serial_number_ = std::string(camlist.at(0).c_str());
          this->np_.setParam("serial_number", this->serial_number_);
        }
      else
        {
          // see if the specific camera was detected
          auto result = std::find(std::begin(camlist), std::end(camlist),
                                  this->serial_number_);
          if (result != std::end(camlist))
            {
              // the specific camera is available
              this->cam_ = manager.createCamera(*result);
            }
          else
            {
              // the specific camera is not available
              NODELET_WARN_STREAM("Could not find royale camera: "
                                  << this->serial_number_);
            }
        }
    }
  else
    {
      NODELET_WARN_STREAM("No royale cameras found on bus!");
    }

  if (this->cam_ != nullptr)
    {
      if (this->cam_->initialize() != royale::CameraStatus::SUCCESS)
        {
          NODELET_INFO_STREAM("Failed to initialize() camera: "
                              << this->serial_number_);
          this->cam_.reset();
        }
      else
        {
          NODELET_INFO_STREAM("Instantiated royale camera: "
                              << this->serial_number_);

          royale::CameraAccessLevel level;
          if (this->cam_->getAccessLevel(level) ==
              royale::CameraStatus::SUCCESS)
            {
              this->access_level_ = (std::uint32_t) level;
            }
          NODELET_INFO_STREAM("Access level: " << this->access_level_);

          //
          // we only create our image publishers once regardless
          // of how many times the node polls the bus for a camera
          //
          if (! this->instantiated_publishers_)
            {
              //
              // Dynamically create our image publishers based on the max
              // number of streams available across all camera use-cases
              //
              royale::Vector<royale::String> use_cases;
              auto status = this->cam_->getUseCases(use_cases);
              if (status == royale::CameraStatus::SUCCESS)
                {
                  std::uint32_t max_num_streams = 1;
                  for(auto& uc : use_cases)
                    {
                      std::uint32_t nstreams = 0;
                      if (this->cam_->getNumberOfStreams(uc, nstreams) !=
                          royale::CameraStatus::SUCCESS)
                        {
                          NODELET_WARN_STREAM("Could not get stream count: "
                                              << uc.c_str());
                        }
                      else
                        {
                          if (nstreams > max_num_streams)
                            {
                              max_num_streams = nstreams;
                            }
                        }
                    } // end: for(auto& uc : use_cases)

                  NODELET_INFO_STREAM("Max number of streams: "
                                      << max_num_streams);
                  for (std::uint32_t i = 0; i < max_num_streams; ++i)
                    {
                      this->exposure_pubs_.push_back(
                        this->np_.advertise<royale_ros::ExposureTimes>(
                          "stream/" + std::to_string(i+1) +
                          "/exposure_times", 1));

                      this->cloud_pubs_.push_back(
                        this->np_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
                          "stream/" + std::to_string(i+1) + "/cloud", 1));

                      this->xyz_pubs_.push_back(
                        this->it_->advertise(
                          "stream/" + std::to_string(i+1) + "/xyz", 1));

                      this->noise_pubs_.push_back(
                        this->it_->advertise(
                          "stream/" + std::to_string(i+1) + "/noise", 1));

                      this->gray_pubs_.push_back(
                        this->it_->advertise(
                          "stream/" + std::to_string(i+1) + "/gray", 1));

                      this->conf_pubs_.push_back(
                        this->it_->advertise(
                          "stream/" + std::to_string(i+1) + "/conf", 1));
                    }

                  this->instantiated_publishers_ = true;
                }
            }

          this->cam_->registerDataListener(this);
          this->cam_->startCapture();
          {
            std::lock_guard<std::mutex> lock(this->last_frame_mutex_);
            this->last_frame_ = ros::Time::now();
          }
        }
    }

  this->RescheduleTimer();
}

void
royale_ros::CameraNodelet::RescheduleTimer()
{
  this->timer_.stop();
  this->timer_.setPeriod(ros::Duration(this->poll_bus_secs_));
  this->timer_.start();
}

bool
royale_ros::CameraNodelet::GetUseCases(royale_ros::GetUseCases::Request& req,
                                       royale_ros::GetUseCases::Response& resp)
{
  std::lock_guard<std::mutex> lock(this->cam_mutex_);
  if (this->cam_ == nullptr)
    {
      NODELET_ERROR_STREAM("No camera instantiated with serial number: "
                           << this->serial_number_);
      return false;
    }

  royale::Vector<royale::String> use_cases;
  auto status = this->cam_->getUseCases(use_cases);
  if (status != royale::CameraStatus::SUCCESS)
    {
      NODELET_ERROR_STREAM((int) status
                           << ": " << royale::getErrorString(status));
      return false;
    }

  std::transform(use_cases.begin(), use_cases.end(),
                 std::back_inserter(resp.use_cases),
                 [](royale::String& s) -> std::string
                 { return std::string(s.c_str()); });
  return true;
}

void
royale_ros::CameraNodelet::onNewData(const royale::DepthData *data)
{
  NODELET_INFO_STREAM("-- PING --");
  {
    std::lock_guard<std::mutex> lock(this->last_frame_mutex_);
    this->last_frame_ = ros::Time::now();
  }

}

PLUGINLIB_EXPORT_CLASS(royale_ros::CameraNodelet, nodelet::Nodelet)
