Building and Installing the royale-ros Package
----------------------------------------------
`royale-ros` is a wrapper around pmd's Royale SDK. To that
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

`royale-ros` is distributed as a [catkin](http://wiki.ros.org/catkin)
package. If you are already comfortable with `catkin`, you should be able to
install it in the usual way. You will not have to read any further. If you
would like step-by-step instructions for one plausible way of using `catkin` to
install `royale-ros`, please keep reading.

# Step-by-Step catkin build and installation instructions

First, we need to decide where we want our software to ultimately be
installed. For purposes of this document, we will assume that we will install
our ROS packages at `~/ros`. For convenience, we add the following to our
`~/.bash_profile`:

```
if [ -f /opt/ros/kinetic/setup.bash ]; then
  source /opt/ros/kinetic/setup.bash
fi

cd ${HOME}

export LPR_ROS=${HOME}/ros

if [ -d ${LPR_ROS} ]; then
    for i in $(ls ${LPR_ROS}); do
        if [ -d ${LPR_ROS}/${i} ]; then
            if [ -f ${LPR_ROS}/${i}/setup.bash ]; then
                source ${LPR_ROS}/${i}/setup.bash --extend
            fi
        fi
    done
fi
```

Next, we need to get the code from github. We assume we keep all of our git
repositories in `~/dev`.

```
$ cd ~/dev
$ git clone https://github.com/lovepark/royale-ros.git
```

We now have the code in `~/dev/royale-ros`. Next, we want to create a _catkin
workspace_ that we can use to build and install that code from. It is the
catkin philosophy that we do not do this directly in the source directory.

```
$ cd ~/catkin
$ mkdir royale
$ cd royale
$ mkdir src
$ cd src
$ catkin_init_workspace
$ ln -s ~/dev/royale-ros royale
```

So, you should have a catkin workspace set up to build the royale-ros code that
looks basically like:

```
[ ~/catkin/royale/src ]
tpanzarella@tuna: $ pwd
/home/tpanzarella/catkin/royale/src

[ ~/catkin/royale/src ]
tpanzarella@tuna: $ ls -l
total 0
lrwxrwxrwx 1 tpanzarella tpanzarella 50 Jul 21 11:36 CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
lrwxrwxrwx 1 tpanzarella tpanzarella 33 Jul 21 11:37 royale -> /home/tpanzarella/dev/royale-ros/
```

Now we are ready to build the code.

```
$ cd ~/catkin/royale
$ catkin_make
$ catkin_make -DCMAKE_INSTALL_PREFIX=${LPR_ROS}/royale install
```

The ROS package should now be installed in `~/ros/royale`. To test everything
out (assuming you have your camera plugged in) you should open a fresh bash
shell, and start up a ROS core:

```
    $ roscore
```

Open another shell and start the camera node:

```
    $ roslaunch royale_ros camera.launch
```

Open another shell and start the rviz node to visualize the data coming from
the camera:

```
    $ roslaunch royale_ros rviz.launch
```

At this point, you should see an rviz window that looks something like:

![rviz1](doc/figures/rviz_screenshot.png)

Congratulations! You can now utilize royale-ros.
