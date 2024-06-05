# Micro ROS Code for the RPI2040 

This repo is a working, empty project. Put the seahawk code in `src/pidrone/pidrone.c`. Supporting code is checked in as a submodule. 

## Checkout 

After checking out this repo you must: 

```
git submodule init 
git submodule update --init --recursive
```

There are lots and lots of subrepos. 

## Building in vscode 

You may have to scan for kits and select a kit as shown in the [Getting started with RPi Pico Instructions](https://ubuntu.com/blog/getting-started-with-micro-ros-on-raspberry-pi-pico)

## Uploading to the Pi Pico 

The binary is in `build/pidrone.uf2`. You can install it by: 

```console
cp build/pidrone.uf2 /media/$USER/RPI-RP2
```
