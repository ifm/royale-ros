royale-ros
==========
royale-ros is a wrapper around the [pmd](http://www.pmdtec.com/) Royale SDK
enabling the usage of [pmd-based ToF cameras](http://pmdtec.com/picofamily/)
from within [ROS](http://www.ros.org/) software systems.

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

Building and Installing the Software
------------------------------------
As stated above, `royale-ros` is a wrapper around pmd's Royale SDK. To that
end, the Royale SDK needs to be installed on your system. Due to licensing
concerns, you will need to acquire the Royale software directly from
pmd. [Here](http://pmdtec.com/picofamily/software/) is a link to their software
download page (password protected -- contact pmd directly for a customer
password). Once you have acquired the binary SDK (typically in a file called
`libroyale.zip`), you can either install it according to the instructions
provided with it, or follow [our instructions](doc/royale_install.md) which
allow you to integrate it with your package manager (assuming you are on a
Debian-based system like Ubuntu). Once you have installed Royale, continue on
with the instructions below to build and install `royale-ros`.

Building and installing `royale-ros` is done via
[catkin](http://wiki.ros.org/catkin). The following step-by-step instructions
should get you up-and-running quickly -- we realize there are various ways of
doing this, one viable way now follows.

TODO... provide build and install instructions


TODO
----
Please see the [Github Issues](https://github.com/lovepark/royale-ros/issues).

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
