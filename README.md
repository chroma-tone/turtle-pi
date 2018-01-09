Turtle-Pi - A Raspberry Pi powered turtlebot style robot for experimentation
---

Setup
---
* The operating system is based on Ubuntu Xenial Server to avoid the time required for installation of ROS on Raspbian.
* Download Ubuntu Mate for Raspberry Pi from https://ubuntu-mate.org/download/, unzip, and write to a MicroSD. 
* I'm on a Mac, so I use [Apple PI Baker](http://mac.softpedia.com/get/Utilities/ApplePi-Baker.shtml)
* Might need to connect monitor, mouse, and keyboard for initial setup.
* Set up a user called ubuntu, and use this for day to day operation
* Recommend use of ssh keys for login to avoid typing password too frequently

* Use raspi-config to configure to boot to cli, and enable ssh (TODO: Details)
* Enabled ssh as follows:

```bash
$ sudo touch /boot/ssh
$ sudo apt update && sudo apt upgrade
```

* Then install ROS as per http://wiki.ros.org/kinetic/Installation/Ubuntu

* And install some basic needs:
```
$ sudo apt install git tmux vim
```

* TODO: Clone repo from github etc
* TODO: Set up WiFi using wpa_supplicant.conf

* TODO: Sources for hardware

Starting up the Robot
---
* ssh into the Turtlebot, and:
```
$ roslaunch turtle_pi turtle_pi.launch
```

* Read this if you're new to tmux: https://robots.thoughtbot.com/a-tmux-crash-course
* In the first terminal, launch the overall application as follows



* Install [ROS Teleop](https://play.google.com/store/apps/details?id=com.github.rosjava.android_apps.teleop.indigo&hl=en) on an Android device (Indigo seems to be the latest version)


Development techniques and Recommended toolchain
---

* TODO: Summary of using sshfs and VSCode for development
* TODO: Use grip to preview markdown: https://github.com/joeyespo/grip
* TODO: Using tmux to keep sessions going



* TODO: Overall Architectuer
* TODO: Links to relevant References
* TODO: Link to Onshape Design files: https://cad.onshape.com/documents/4f29d05b378048f3037980f1/w/bcd3158716b8d440662c4aa1/e/be55f8279a491f57b7430f17

Camera Setup / Install
---

```
$ sudo apt install ros-kinetic-compressed-image-transport  
$ sudo apt install ros-kinetic-camera-info-manager-py # Maybe wasn't required?
$ sudo apt install libcamera-info-manager-dev # Maybe wasn't required?
$ sudo apt install ros-kinetic-camera-info-manager

```

* --Then install raspicam-node as per https://github.com/dganbold/raspicam_node--

Nope - That one doesn't provide compressed image transport (only raw)

Trying the other one: https://github.com/Dhivin/raspicam_node-1

OK - Looks good - ROS Teleop is subscribing on /compressed_image

Need to execute:

```
$ roslaunch turtle_pi turtle_pi.launch
```

Can use rqt_image_viewer to preview image from turtlepi
