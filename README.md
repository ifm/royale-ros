royale-ros
==========
royale-ros is a wrapper around the [pmd](http://www.pmdtec.com/) Royale SDK
enabling the usage of [pmd-based ToF cameras](http://pmdtec.com/picofamily/)
from within [ROS](http://www.ros.org/) software systems.

![rviz1](doc/figures/rviz_screenshot.png)

Software Compatibility Matrix
=============================

<table>
  <tr>
    <th>royale-ros version</th>
    <th>Royale SDK version</th>
    <th>Linux/ROS distribution pair(s)</th>
    <th>Supported Hardware</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>3.5</td>
    <td>
      Ubuntu 16.04/Kinetic,<br/>
      Ubuntu 14.04/Indigo
    </td>
    <td>Pico Flexx</td>
  </tr>
  <tr>
    <td>0.2.0</td>
    <td>3.16.0.51</td>
    <td>
      Ubuntu 16.04/Kinetic
    </td>
    <td>
      Pico Flexx,<br/>
      Pico Monstar
    </td>
  </tr>
</table>

**NOTE:** Theoretically, any camera supported by Royale will be compatible with
  this library. However, the above listed hardware is what we have available to
  us for testing. We welcome your feedback related to other Royale-based
  cameras and their compatibility with this ROS interface.

Building and Installing the Software
====================================
Building and installing the software has two primary steps:

1. [Installing the pmd Royale SDK](doc/royale_install.md)
2. [Installing the ROS node](doc/building.md)

ROS Interface
=============

## camera nodelet

The core `royale-ros` sensor interface is implemented as a ROS nodelet. This
allows for lower-latency data processing vs. the traditional out-of-process
node-based ROS interface for applications that require it. However, we ship a
launch file with this package that allows for using the core `royale-ros`
driver as a standard node. To launch the node the following command can be
used:

```
$ roslaunch royale_ros camera.launch
```

This launch file encapsulates several features:

1. It exposes some of the `camera_nodelet` parameters as command-line arguments
   for ease of runtime configuration.
2. It instantiates a nodelet manager which the `camera_nodelet` will be loaded
   into.
3. It launches the `camera_nodelet` itself.
4. It publishes the static transform from the camera's optical frame to a
   traditional ROS sensor frame as a tf2 `static_transform_publisher`.

You can either use [this launch file](launch/camera.launch) directly, or, use
it as a basis for integrating `royale-ros` into your own robot software
system.

### Parameters

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>~access_code</td>
    <td>string</td>
    <td>-</td>
    <td>
      The Royale access code used to unlock level 2+ access to the Royale SDK
      functions. If no access code is supplied (the default), Level 1 access
      will be granted by Royale.
    </td>
  </tr>
  <tr>
    <td>~on_at_startup</td>
    <td>bool</td>
    <td>true</td>
    <td>
      The pmd cameras use an active illumination unit as part of its ToF
      measurement principle to compute the distance to objects in the
      scene. Continuously pulsing the illumination unit both consumes power and
      generates heat. To that end, for some applications, it is desireable to
      have the ability to turn on/off the illumination unit in
      software. royale-ros provides that capability through a ROS service call
      (see below). This parameter controls whether or not the camera will start
      pulsing its illumination unit (and by extension, streaming image data) at
      node startup time.
    </td>
  </tr>
  <tr>
    <td>~serial_number</td>
    <td>string</td>
    <td>-</td>
    <td>
      Each pmd camera has a unique serial number. Each instance of the
      royale-ros camera nodelet manages the data from a specific camera
      instance. This parameter controls which specific camera serial number
      this instance of the nodelet should manage. The special string "-" (a
      minus sign, no quotes) communicates to the nodelet that the first camera
      found on the USB bus should be used. This is convenient for cases where
      you will only ever be using a single camera but may be using many
      different cameras with different serial numbers. At the same time,
      having this serial number mapping allows for robust robot configurations
      whereby you can map camera serial numbers to semantically meaningful node
      names (e.g., serial number XXX = front_left_camera).
    </td>
  </tr>
  <tr>
    <td>~initial_use_case</td>
    <td>string</td>
    <td>-</td>
    <td>
      pmd-based Royale cameras encapsulate a set of camera and imager settings
      into the notion of a "use case". These use cases give a name to a set of
      prepackaged parameter settings. This parameter allows for setting a
      particular use case on the camera at nodelet startup time. The use case
      can be changed at any point in time via the Config service, however, this
      simplifies setting the use case at startup. The special (and default) value
      of "-" (a minus sign, no quotes) communicates to the nodelet that no
      particular use case should be set at startup time.
    </td>
  </tr>
  <tr>
    <td>~poll_bus_secs</td>
    <td>float</td>
    <td>1.</td>
    <td>
      Integral to the camera nodelet is a running watchdog timer. This
      parameter controls the frequency at which the watchdog will run a health
      check loop. This is also the mechanism by which this nodelet provides
      robustness to a camera cable being unplugged and re-plugged back in
      (something that could very likely happen in an industrial setting due to
      pinched cables, etc.). Every poll_bus_secs this node will try to
      reinitialize the camera should it become unplugged for whatever reason.
    </td>
  </tr>
  <tr>
    <td>~timeout_secs</td>
    <td>float</td>
    <td>1.</td>
    <td>
      This is a threshold value used by the nodelet to determine that a running
      camera has timed-out or otherwise become unavailable. On every iteration
      of the watchdog (i.e., every poll_bus_secs), if the camera is currently
      "on", a check is made to see if frame data have been received within this
      timeout threshold. If this timeout threshold has been exceeded, the
      camera will be reinitialized.
    </td>
  </tr>
  <tr>
    <td>~optical_frame</td>
    <td>string</td>
    <td>camera_optical_link</td>
    <td>The name of the optical frame in the tf tree</td>
  </tr>
  <tr>
    <td>~sensor_frame</td>
    <td>string</td>
    <td>camera_link</td>
    <td>The name of the sensor frame in the tf tree</td>
  </tr>
</table>

### Published Topics

**NOTE:** pmd cameras can produce data with different imager settings
  simulataneously. This is the so-called *mixed mode* feature. For a particular
  use case there will be one or more data *streams* that it offers. So for each
  image topic published by `royale-ros`, we namespace it into a stream. While
  Royale gives each camera stream a unique *stream id* encoded as a `uint16`,
  for the purpose of topic names, `royale-ros` streams are simply positive
  integers. So, for example, the point cloud topic for a single stream use case
  will be published on `stream/1/cloud`. For a mixed-mode use case (lets assume
  two data streams), the point clouds will be published on `stream/1/cloud` and
  `stream/2/cloud`. In the table below, we use the variable `X` as a
  placeholder for the stream number.

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>stream/X/camera_info</td>
    <td>sensor_msgs/CameraInfo</td>
    <td>The intrinsic calibration parameters for the camera</td>
  </tr>
  <tr>
    <td>stream/X/cloud</td>
    <td>sensor_msgs/PointCloud2</td>
    <td>The point cloud data</td>
  </tr>
  <tr>
    <td>stream/X/conf</td>
    <td>sensor_msgs/Image</td>
    <td>The pixel confidence image</td>
  </tr>
  <tr>
    <td>stream/X/gray</td>
    <td>sensor_msgs/Image</td>
    <td>The amplitude image</td>
  </tr>
  <tr>
    <td>stream/X/noise</td>
    <td>sensor_msgs/Image</td>
    <td>The noise image</td>
  </tr>
  <tr>
    <td>stream/X/xyz</td>
    <td>sensor_msgs/Image</td>
    <td>
      The point cloud data (Cartesian data only, no amplitude) encoded as a
      three channel (the x, y, z spatial planes respectively) OpenCV image.
    </td>
  </tr>
  <tr>
    <td>stream/X/exposure_times</td>
    <td><a href="msg/ExposureTimes.msg">royale_ros/ExposureTimes</a></td>
    <td>The exposure times used to acquire the pixel data.</td>
  </tr>
</table>

### Subscribed Topics

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>SetExposureTime</td>
    <td><a href="msg/SetExposureTime.msg">royale_ros/SetExposureTime</a></td>
    <td>
      Allows for a lightweight/fast means to change the exposure time for the
      current use case on-the-fly (assuming the camera is in a manual exposure
      mode).
    </td>
  </tr>
</table>

### Advertised Services

<table>
  <tr>
    <th>Name</th>
    <th>Service Definition</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>Dump</td>
    <td><a href="srv/Dump.srv">royale_ros/Dump</a></td>
    <td>Dumps the state of the camera parameters to JSON</td>
  </tr>
  <tr>
    <td>Config</td>
    <td><a href="srv/Config.srv">royale_ros/Config</a></td>
    <td>
      Provides a means to configure the camera and imager settings,
      declaratively from a JSON encoding of the desired settings.
    </td>
  </tr>
  <tr>
    <td>Start</td>
    <td><a href="srv/Start.srv">royale_ros/Start</a></td>
    <td>Starts the camera data stream</td>
  </tr>
  <tr>
    <td>Stop</td>
    <td><a href="srv/Stop.srv">royale_ros/Stop</a></td>
    <td>
      Stops the camera data stream and, by extension, turns off the active
      illumination unit.
    </td>
  </tr>
</table>

Additional Documentation
========================

* [Listing serial numbers of currently connected cameras](doc/lscam.md)
* [Inspecting and configuring the camera/imager settings](doc/dump_and_config.md)
* [Changing the exposure time on the fly](doc/changing_exposures.md)
* [Handling multiple cameras](doc/multiple_cameras.md)

TODO
====
The current TODO list is located [here](TODO.md). Please also see the
[Github Issues](https://github.com/lovepark/royale-ros/issues).

LICENSE
=======
Please see the file called [LICENSE](LICENSE).

ATTRIBUTION
===========
The initial development of `royale-ros` has been sponsored by
[Locus Robotics](http://www.locusrobotics.com/). The authors thank them for
their contribution to the open-source robotics community.

AUTHORS
=======
Tom Panzarella <tom@loveparkrobotics.com>

<p align="center">
  <br/>
  <img src="doc/figures/LPR_logo_fullcolor.png"/>
  <br/>
  Copyright &copy; 2017 Love Park Robotics, LLC
</p>
