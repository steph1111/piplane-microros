# PiPlane/Drone DSHOT-MicroROS Code for the RPI2040(Pi Pico)
This repo is a working, empty project. Put the seahawk code in `src/piplane/piplane.c`. Supporting code is checked in as a submodule. 

## Checkout 

After checking out this repo you must: 

```
git submodule init 
git submodule update --init --recursive
```

## Building in vscode 

You may have to scan for kits and select a kit as shown in the [Getting started with RPi Pico Instructions](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico)

## Uploading to the Pi Pico 

The binary is in `build/piplane.uf2`. You can install it by: 

```console
cp build/pidrone.uf2 /media/$USER/RPI-RP2
```
## ROS2 Integration
The Pico will expect a ROS2 topic called **/motor_array**
Currently this is expected to be an int16_multi_array, with a length of 4. However, this could be extended for use cases with more motors. You can test this by artifically publishing in terminal:
```console
ros2 topic pub /motor_array std_msgs/msg/Int16MultiArray "{data: [0, 0, 0, 0]}"
```
Note that DSHOT Expects a throttle value between 0 & 2047. It can also get a little fussy if you start with a value other than 0.