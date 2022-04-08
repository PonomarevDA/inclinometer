# Inclinometer

This is ROS-package that allows you to visualize inclinometer position and orientation of connected via UAVCAN inclinometer.

## Requirements

- ubuntu 18.04
- ros melodic

## How to use it

The easiest way to play with this package is to use [scripts/docker.sh](scripts/docker.sh) script.

Try `./scripts/docker.sh --help` to get detailed info.

Typically you need 3 commands:
- `./scripts/docker.sh build` to build the docker image
- `./scripts/docker.sh run` to run the docker container
- `./scripts/docker.sh kill` to kill all existed containers (just in case if your container unsuccessfully finished, sometimes it happens yet).

## Example

Check the video below:

[![inclinometer_usage](https://img.youtube.com/vi/GTkq55MVPJ0/0.jpg)](https://youtu.be/GTkq55MVPJ0)
