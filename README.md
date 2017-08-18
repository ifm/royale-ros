royale-ros
==========
royale-ros is a wrapper around the [pmd](http://www.pmdtec.com/) Royale SDK
enabling the usage of [pmd-based ToF cameras](http://pmdtec.com/picofamily/)
from within [ROS](http://www.ros.org/) software systems.

![rviz1](doc/figures/rviz_screenshot.png)

Software Compatibility Matrix
-----------------------------

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
    <td>Ubuntu 16.04/Kinetic</td>
    <td>Pico Flexx</td>
  </tr>
</table>

**NOTE:** Theoretically, any camera supported by Royale will be compatible with
  this library. However, the above listed hardware is what we have available to
  us for testing. We welcome your feedback related to other Royale-based
  cameras and their compatibility with this ROS interface.

**NOTE 2:** This library is (currently) limited to Royale's Level 1 access
  features.

Building and Installing the Software
------------------------------------
Building and installing the software has two primary steps:

1. [Installing the pmd Royale SDK](doc/royale_install.md)
2. [Installing the ROS node](doc/building.md)


TODO
----
The current TODO list is located [here](TODO.md). Please also see the
[Github Issues](https://github.com/lovepark/royale-ros/issues).

LICENSE
-------
Please see the file called [LICENSE](LICENSE).

ATTRIBUTION
-----------
The initial development of `royale-ros` has been sponsored by
[Locus Robotics](http://www.locusrobotics.com/). The authors thank them for
their contribution to the open-source robotics community.

AUTHORS
-------
Tom Panzarella <tom@loveparkrobotics.com>

<p align="center">
  <br/>
  <img src="doc/figures/LPR_logo_fullcolor.png"/>
  <br/>
  Copyright &copy; 2017 Love Park Robotics, LLC
</p>
