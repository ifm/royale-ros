royale-ros TODO
---------------

* Enumerate the features of the software in the REAME
* Create comprehensive documentation for ROS users
* Add an OpenCV encoding of the point cloud data (3 spatial planes)
* Publish the camera intrinsics on a `camera_info` topic
* Add a `Dump` service wrapper similar to `Config`
* Performance enhancements
  * In-line transformation of the point cloud to the sensor frame while we are
    parsing / constructing the data. This will save users some cycles by not
    having to transform the points from the optical frame. This is particulary
    helpful when operating at the high frame-rates.
* Expose a topic for changing exposure times on the fly
* Provide a service to shutdown the camera stream (do these Royale cameras get
  hot if pulsing the illumination unit all day?)
* Multi-camera testing
* Testing on Ubuntu 14.04 and ROS Indigo
* Provide a means to set the initial use case at the time of launching the
  camera node. Currently, after launching the camera, to set the proper use
  case the `Config` service needs to be called.
* Endurance testing -- have heard complaints about USB stability, particularly
  among industrial users. We need to test to see if the integral _watchdog_ we
  currently have implemented is enough or if we need to add additional levels
  of robustness. Currently, you should be able to plug/unplug a camera willy
  nilly and the node should self heal.