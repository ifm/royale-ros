Manually changing the exposure time on a running camera
=======================================================

This article explains the facilities provided by `royale-ros` to manually
change the exposure times on the fly for a running camera. Changing the
exposure is set on a per stream basis.

The first step is to inspect our current camera configuration.

```
$ rosrun royale_ros dump
{
  "Device": {
    "Id": "0005-4804-0050-1916",
    "Name": "PICOFLEXX"
  },
  "Imager": {
    "CurrentUseCase": {
      "ExposureLimits": {
        "57082": [
          "27",
          "2000"
        ]
      },
      "ExposureMode": {
        "57082": "0"
      },
      "FrameRate": "5",
      "MaxFrameRate": "5",
      "Name": "MODE_9_5FPS_2000",
      "NumberOfStreams": "1",
      "Streams": [
        "57082"
      ]
    },
    "MaxSensorHeight": "171",
    "MaxSensorWidth": "224",
    "UseCases": [
      "MODE_9_5FPS_2000",
      "MODE_9_10FPS_1000",
      "MODE_9_15FPS_700",
      "MODE_9_25FPS_450",
      "MODE_5_35FPS_600",
      "MODE_5_45FPS_500",
      "MODE_MIXED_30_5",
      "MODE_MIXED_50_5"
    ]
  }
}
```

The first thing to note is that the imager is configured to use a single stream
`57082`. Next, we see that this particular stream is set to a manual
`ExposureMode`:

```
"ExposureMode": {
    "57082": "0"
 },
```

**NOTE:** manual exposure = 0, auto exposure = 1.

We also note the exposure limits for this stream:

```
"ExposureLimits": {
    "57082": [
      "27",
      "2000"
    ]
},
```

The units of the exposure limits are in microseconds. Let's now see what the
camera is reporting for the *actual* exposure times used:

```
$ rostopic echo /camera/stream/1/exposure_times
header:
  seq: 1536
  stamp:
    secs: 1503082272
    nsecs: 754138947
  frame_id: camera_optical_link
usec: [200, 2000, 2000]
---
^C
```

Lets now set the long exposure to 1000 usecs. This is done by publishing on the
`SetExposureTime` topic:

```
$ rostopic pub -1 /camera/SetExposureTime royale_ros/SetExposureTime "streamid: 57082
exposure_usecs: 1000"
publishing and latching message for 3.0 seconds
```

Now, let's see what the camera is reporting:

```
$ rostopic echo /camera/stream/1/exposure_times
header:
  seq: 2480
  stamp:
    secs: 1503082461
    nsecs: 649880886
  frame_id: camera_optical_link
usec: [200, 1000, 1000]
---
^C
```

If your `camera_nodelet` is running with level 2 access, you will have access
to finer-grained control over all of the exposure times via the
`/camera/SetExposureTimes` service:

```
$ rostopic pub -1 /camera/SetExposureTimes royale_ros/SetExposureTimes "streamid: 0
exposure_usecs: [100, 1000, 1750]"
```

And,

```
$ rostopic echo /camera/stream/1/exposure_times
header:
  seq: 5528
  stamp:
    secs: 1526690890
    nsecs: 376950026
  frame_id: "camera_optical_link"
usec: [100, 1000, 1750]
---
^C
```

To be clear, the output from the `Dump` service will report the exposure limits
for the configured use case. The `stream/X/exposure_times` topic will publish
the actual exposure times used for image acquisition.
