Discovering camera serial numbers
=================================

`royale-ros` provides a command line tool for listing all connected Royale
cameras and their serial numbers. The tool to list the connected cameras is
called `lscam`. Its usage now follows.

```
$ rosrun royale_ros lscam
[
  "0005-1212-0034-2109",
  "0005-4804-0050-1916"
]
```

We see in our example above, we have two connected cameras. We also note that
if a running instance of the `camera_nodelet` is managing a particular camera,
it will not show up in this list. So, to get a global list of all connected
cameras be sure that no instances of the `camera_nodelet` is running.
