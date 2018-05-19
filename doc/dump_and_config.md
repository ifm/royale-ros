royale-ros: Inspecting and configuring the camera/imager settings
=================================================================

`royale-ros` provides access to the camera/imager parameters via the `Dump` and
`Config` services on the camera nodelet. The approach employed by `royale-ros`
is to encode the settings via a JSON serialization. To inpsect the settings you
use the `Dump` service to get a JSON encoding of the parameters. To mutate the
settings you use the `Config` service by providing a JSON serialization of your
*desired* settings. Specifics on how to use these services now follows.

**NOTE:** In some of the examples that follow, we use the
  [jq](https://stedolan.github.io/jq/) command line tool to process the
  `royale-ros` JSON stream via a Linux pipeline. If you are on Ubuntu, you can
  get `jq` via `sudo apt-get install jq`.

## The Dump service

To dump the camera settings, you can directly call the `Dump` service via
`rosservice` or call the service from your own code and process the JSON
there. Using the `rosservice` command line tool, this looks like:

```
$ rosservice call /camera/Dump
config: {
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

While calling the `Dump` service via `rosservice` is easy and convenient, it is
non-optimal as `rosservice`'s own mark-up (see the `config:` at the begining of
the output?) produces invalid JSON for processing as part of a larger Linux
pipeline. To that end, we provide a convenience tool to be used as part of a
pipeline. To produce the same output but while emitting valid JSON you can:

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

This can now very easily be used in a pipeline. For example, you could copy the
parameters from one running camera into another running camera by piping this
output into the `Config` service of another running camera instance. For
exemplary purposes, we will show a simple example of extracting the valid use
cases you can set on the camera -- (yes, we know, you can easily see them
above, however, this is here to show you the intent of processing these JSON
data as part of a larger pipeline or to be used in scripts that manage fleets
of robots or fleets of cameras on a single robot or fleets of cameras
distributed over a fleet of robots -- you get the point).

```
$ rosrun royale_ros dump | jq .Imager.UseCases
[
  "MODE_9_5FPS_2000",
  "MODE_9_10FPS_1000",
  "MODE_9_15FPS_700",
  "MODE_9_25FPS_450",
  "MODE_5_35FPS_600",
  "MODE_5_45FPS_500",
  "MODE_MIXED_30_5",
  "MODE_MIXED_50_5"
]
```

We note, that the examples above show exemplary output with Royale level 1
access. If you started the `camera_nodelet` with a level 2 (or greater) access
code, the Royale processing parameters will also be exposed and are
tunable. For completeness, a level dump from a Pico Flexx may look like this:

```
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
      "ProcessingParameters": {
        "57082": {
          "AdaptiveNoiseFilterType_Int": "1",
          "AutoExposureRefValue_Float": "1000.000000",
          "ConsistencyTolerance_Float": "1.200000",
          "FlyingPixelsF0_Float": "0.018000",
          "FlyingPixelsF1_Float": "0.140000",
          "FlyingPixelsFarDist_Float": "4.500000",
          "FlyingPixelsNearDist_Float": "1.000000",
          "GlobalBinning_Int": "1",
          "LowerSaturationThreshold_Int": "400",
          "MPIAmpThreshold_Float": "0.300000",
          "MPIDistThreshold_Float": "0.100000",
          "MPINoiseDistance_Float": "3.000000",
          "NoiseThreshold_Float": "0.070000",
          "UpperSaturationThreshold_Int": "3750",
          "UseAdaptiveBinning_Bool": "false",
          "UseAdaptiveNoiseFilter_Bool": "true",
          "UseAutoExposure_Bool": "false",
          "UseFilter2Freq_Bool": "true",
          "UseMPIFlagAverage_Bool": "true",
          "UseMPIFlag_Amp_Bool": "true",
          "UseMPIFlag_Dist_Bool": "true",
          "UseRemoveFlyingPixel_Bool": "true",
          "UseRemoveStrayLight_Bool": "false",
          "UseSparsePointCloud_Bool": "false",
          "UseValidateImage_Bool": "true"
        }
      },
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

## The Config service

To change camera/imager parameters, the `Config` service is used. The `Config`
service accepts the exact same JSON that the `Dump` service emits, however, it
interprets that JSON as a *desired state* for the camera. Said another way,
this allows for *declaratively* mutating camera parameters. We also note that
the whole JSON encoding need not be specified (we will exploit this in the
example below).

For exemplary purposes, we will use the command line to change the current use
case on the camera. Let's baseline the camera's current state by inspecting the
dump:

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

As a first step to changing the use case, let's first discover the current use
case that is active on the camera:

```
$ rosrun royale_ros dump | jq .Imager.CurrentUseCase.Name
"MODE_9_5FPS_2000"
```

Next, let's see what valid use cases we can set on the camera

```
$ rosrun royale_ros dump | jq .Imager.UseCases
[
  "MODE_9_5FPS_2000",
  "MODE_9_10FPS_1000",
  "MODE_9_15FPS_700",
  "MODE_9_25FPS_450",
  "MODE_5_35FPS_600",
  "MODE_5_45FPS_500",
  "MODE_MIXED_30_5",
  "MODE_MIXED_50_5"
]
```

For fun, let's apply one of the mixed mode uses cases to the camera:

```
$ echo '{"Imager":{"CurrentUseCase":{"Name":"MODE_MIXED_30_5"}}}' | rosrun royale_ros config
[ INFO] [1503078647.907152832]: status=0
[ INFO] [1503078647.907353149]: msg=OK
```

For clarity, all we needed to do was pipe in a valid JSON snippet to set the
new use case on the camera.

We now check to see that our camera setting did in fact get set on the camera:

```
$ rosrun royale_ros dump | jq .Imager.CurrentUseCase.Name
"MODE_MIXED_30_5"
```

And, for completeness, let's look at the whole dump (you will note there are
now two streams encoded in the dump):

```
{
  "Device": {
    "Id": "0005-4804-0050-1916",
    "Name": "PICOFLEXX"
  },
  "Imager": {
    "CurrentUseCase": {
      "ExposureLimits": {
        "57082": [
          "134",
          "300"
        ],
        "57083": [
          "134",
          "1300"
        ]
      },
      "ExposureMode": {
        "57082": "0",
        "57083": "0"
      },
      "FrameRate": "40",
      "MaxFrameRate": "30",
      "Name": "MODE_MIXED_30_5",
      "NumberOfStreams": "2",
      "Streams": [
        "57082",
        "57083"
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

And with level 2 access:

```
{
  "Device": {
    "Id": "0005-4804-0050-1916",
    "Name": "PICOFLEXX"
  },
  "Imager": {
    "CurrentUseCase": {
      "ExposureLimits": {
        "57082": [
          "134",
          "300"
        ],
        "57083": [
          "134",
          "1300"
        ]
      },
      "ExposureMode": {
        "57082": "0",
        "57083": "0"
      },
      "FrameRate": "40",
      "MaxFrameRate": "30",
      "Name": "MODE_MIXED_30_5",
      "NumberOfStreams": "2",
      "ProcessingParameters": {
        "57082": {
          "AdaptiveNoiseFilterType_Int": "1",
          "AutoExposureRefValue_Float": "1000.000000",
          "ConsistencyTolerance_Float": "1.200000",
          "FlyingPixelsF0_Float": "0.018000",
          "FlyingPixelsF1_Float": "0.140000",
          "FlyingPixelsFarDist_Float": "4.500000",
          "FlyingPixelsNearDist_Float": "1.000000",
          "GlobalBinning_Int": "1",
          "LowerSaturationThreshold_Int": "400",
          "MPIAmpThreshold_Float": "0.300000",
          "MPIDistThreshold_Float": "0.100000",
          "MPINoiseDistance_Float": "3.000000",
          "NoiseThreshold_Float": "0.070000",
          "UpperSaturationThreshold_Int": "3750",
          "UseAdaptiveBinning_Bool": "false",
          "UseAdaptiveNoiseFilter_Bool": "true",
          "UseAutoExposure_Bool": "false",
          "UseFilter2Freq_Bool": "false",
          "UseMPIFlagAverage_Bool": "false",
          "UseMPIFlag_Amp_Bool": "false",
          "UseMPIFlag_Dist_Bool": "false",
          "UseRemoveFlyingPixel_Bool": "true",
          "UseRemoveStrayLight_Bool": "false",
          "UseSparsePointCloud_Bool": "false",
          "UseValidateImage_Bool": "true"
        },
        "57083": {
          "AdaptiveNoiseFilterType_Int": "1",
          "AutoExposureRefValue_Float": "1000.000000",
          "ConsistencyTolerance_Float": "1.200000",
          "FlyingPixelsF0_Float": "0.018000",
          "FlyingPixelsF1_Float": "0.140000",
          "FlyingPixelsFarDist_Float": "4.500000",
          "FlyingPixelsNearDist_Float": "1.000000",
          "GlobalBinning_Int": "1",
          "LowerSaturationThreshold_Int": "400",
          "MPIAmpThreshold_Float": "0.300000",
          "MPIDistThreshold_Float": "0.100000",
          "MPINoiseDistance_Float": "3.000000",
          "NoiseThreshold_Float": "0.070000",
          "UpperSaturationThreshold_Int": "3750",
          "UseAdaptiveBinning_Bool": "false",
          "UseAdaptiveNoiseFilter_Bool": "true",
          "UseAutoExposure_Bool": "false",
          "UseFilter2Freq_Bool": "true",
          "UseMPIFlagAverage_Bool": "false",
          "UseMPIFlag_Amp_Bool": "false",
          "UseMPIFlag_Dist_Bool": "false",
          "UseRemoveFlyingPixel_Bool": "true",
          "UseRemoveStrayLight_Bool": "false",
          "UseSparsePointCloud_Bool": "false",
          "UseValidateImage_Bool": "true"
        }
      },
      "Streams": [
        "57082",
        "57083"
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
