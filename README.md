# Inclinometer

This is ROS-package that allows you to visualize inclinometer position and orientation of connected via UAVCAN inclinometer.

## Requirements

- ubuntu 18.04
- ros melodic

## Dependencies

- [uavcan_communicator](https://github.com/InnopolisAero/uavcan_communicator/tree/02f68e80345714dbf2e7e9a9d850cefd5fdc7d0f)

## How to use it

First of all, you need to clone this repository with (important) submodules:

```bash
git clone https://github.com/PonomarevDA/inclinometer --recursive
```

Don't forget to update submodules after each pull. Or call this command if you forget to add `--recursive` in previous command:

```bash
git submodules update --init --recursive
```

The easiest way to play with this package is to use [scripts/docker.sh](scripts/docker.sh) script.

Try `./scripts/docker.sh --help` to get detailed info.

Typically you need 3 commands:
- `./scripts/docker.sh build` to build the docker image
- `./scripts/docker.sh run` to run the docker container
- `./scripts/docker.sh kill` to kill all existed containers (just in case if your container unsuccessfully finished, sometimes it happens yet).

## Example

Check the video below:

[![inclinometer_usage](https://img.youtube.com/vi/GTkq55MVPJ0/0.jpg)](https://youtu.be/GTkq55MVPJ0)
