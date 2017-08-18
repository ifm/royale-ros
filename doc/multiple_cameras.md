Configuring Multiple Cameras
============================

When dealing with multiple cameras, we need to manage the ROS namespace
properly. Additionally, it is desireable for us to create semantically relevant
names for our sensors. We now walk through an example of using two cameras
connected to the same machine.

The first thing we need to do is discriminate between the cameras by serial
number.

```
$ rosrun royale_ros lscam
[
  "0005-1212-0034-2109",
  "0005-4804-0050-1916"
]
```

Lets assume we want to map the camera with serial number `0005-1212-0034-2109`
to the name `left_camera` and the camera with serial number
`0005-4804-0050-1916` to the name `right_camera`.

Let's bring up the left camera:

```
$ roslaunch royale_ros camera.launch serial_number:=0005-1212-0034-2109 camera:=left_camera
... logging to /home/tpanzarella/.ros/log/49cd49bc-8448-11e7-83d9-44850067bbb4/roslaunch-tuna-8180.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://tuna:45369/

SUMMARY
========

PARAMETERS
 * /left_camera/initial_use_case: -
 * /left_camera/on_at_startup: True
 * /left_camera/optical_frame: left_camera_optic...
 * /left_camera/poll_bus_secs: 1.0
 * /left_camera/sensor_frame: left_camera_link
 * /left_camera/serial_number: 0005-1212-0034-2109
 * /left_camera/timeout_secs: 1.0
 * /rosdistro: kinetic
 * /rosversion: 1.12.7

NODES
  /
    left_camera (nodelet/nodelet)
    left_camera_standalone_nodelet (nodelet/nodelet)
    left_camera_tf (tf2_ros/static_transform_publisher)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[left_camera_standalone_nodelet-1]: started with pid [8198]
process[left_camera-2]: started with pid [8199]
process[left_camera_tf-3]: started with pid [8200]
[ INFO] [1503084078.559177248]: Loading nodelet /left_camera of type royale_ros/camera_nodelet to manager left_camera_standalone_nodelet with the following remappings:
[ INFO] [1503084078.566436535]: waitForService: Service [/left_camera_standalone_nodelet/load_nodelet] has not been advertised, waiting...
[ INFO] [1503084078.575066082]: Initializing nodelet with 8 worker threads.
[ INFO] [1503084078.587563108]: waitForService: Service [/left_camera_standalone_nodelet/load_nodelet] is now available.
[ INFO] [1503084078.639667319]: onInit(): /left_camera
[ INFO] [1503084078.648803944]: Probing for available royale cameras...
[ INFO] [1503084081.065769836]: Instantiated royale camera: 0005-1212-0034-2109
[ INFO] [1503084081.065804228]: Access level: 1
[ INFO] [1503084081.065950302]: Max number of streams: 2
[ INFO] [1503084081.358929963]: Caching intrinsic calibration...
[ INFO] [1503084082.022025691]: StreamId cache miss: 57082
```

Now the right:

```
$ roslaunch royale_ros camera.launch serial_number:=0005-4804-0050-1916 camera:=right_camera
... logging to /home/tpanzarella/.ros/log/49cd49bc-8448-11e7-83d9-44850067bbb4/roslaunch-tuna-8658.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://tuna:38370/

SUMMARY
========

PARAMETERS
 * /right_camera/initial_use_case: -
 * /right_camera/on_at_startup: True
 * /right_camera/optical_frame: right_camera_opti...
 * /right_camera/poll_bus_secs: 1.0
 * /right_camera/sensor_frame: right_camera_link
 * /right_camera/serial_number: 0005-4804-0050-1916
 * /right_camera/timeout_secs: 1.0
 * /rosdistro: kinetic
 * /rosversion: 1.12.7

NODES
  /
    right_camera (nodelet/nodelet)
    right_camera_standalone_nodelet (nodelet/nodelet)
    right_camera_tf (tf2_ros/static_transform_publisher)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[right_camera_standalone_nodelet-1]: started with pid [8677]
process[right_camera-2]: started with pid [8678]
process[right_camera_tf-3]: started with pid [8679]
[ INFO] [1503084138.769005867]: Loading nodelet /right_camera of type royale_ros/camera_nodelet to manager right_camera_standalone_nodelet with the following remappings:
[ INFO] [1503084138.777704188]: waitForService: Service [/right_camera_standalone_nodelet/load_nodelet] has not been advertised, waiting...
[ INFO] [1503084138.793439312]: Initializing nodelet with 8 worker threads.
[ INFO] [1503084138.798616444]: waitForService: Service [/right_camera_standalone_nodelet/load_nodelet] is now available.
[ INFO] [1503084138.852212941]: onInit(): /right_camera
[ INFO] [1503084138.861103868]: Probing for available royale cameras...
[ INFO] [1503084141.117220436]: Instantiated royale camera: 0005-4804-0050-1916
[ INFO] [1503084141.117258562]: Access level: 1
[ INFO] [1503084141.117426667]: Max number of streams: 2
[ INFO] [1503084141.402304248]: Caching intrinsic calibration...
[ INFO] [1503084142.058688480]: StreamId cache miss: 57082
```

Lets now see what nodes are running:

```
$ rosnode list
/left_camera
/left_camera_standalone_nodelet
/left_camera_tf
/right_camera
/right_camera_standalone_nodelet
/right_camera_tf
/rosout
```

And, for exemplary purposes, lets see where the point cloud will be published:

```
$ rostopic list | grep -i 'cloud'
/left_camera/stream/1/cloud
/left_camera/stream/2/cloud
/right_camera/stream/1/cloud
/right_camera/stream/2/cloud
```

This short tutorial should provide you with the basis for successfully setting
up `royale-ros`-based multi-camera systems.
