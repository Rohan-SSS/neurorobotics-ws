# Installing pre-equisites

For docker setup we need docker, git and ssh. For non docker setup, we can do away with docker. Also, these scripts have been tested on Ubuntu 20.04.

**Run setup-first-time.sh with desired options**

```
skynet-ws$ ./djinn install --docker --ssh --git
```
Add the generated ssh key to your github account. You may have to reboot to make docker work.


# Docker NRT setup

This simply automates usual installation. It builds all required docker images and installs all dependency libraries, git
pulls all repos and builds them. Tested for Ubuntu 20.04. **BFX and its related dependencies must be manually placed within ext folder in the workspace**

To build Skynet only and Skynet-ROS Images and start their containers run:
```
skynet-ws$ ./djinn init docker
```

To build NRT only image and start the container:
```
skynet-ws$ ./djinn init docker nrt
```

To build Kalibr docker image and start the container:
```
skynet-ws$ ./djinn init docker kalibr
```

# Initialising NRT Docker Container and building inside Docker Container

- **Run `xhost +` command on laptop terminal (no inside docker). This allows Pangolin inside docker to connect host’s X server.**

- Run the following initialisation commands according to need:
    - To initialise **NRT** container: `./djinn up nrt`

- To build ORBSlam2, ORBSlam3, AKAZESlam, NRT and/or NRT-ROS run the following commands according to need:
    - To build all of the above: `./djinn build`
    - To build NRT: `./djinn build nrt`

# Miscellaneous Commands

The following commands are also available through ./djinn:

- To run commands in any initialised docker container: `./djinn exec <Container Suffix> <Command>`
    - For instance to enter a bash shell in the SITL container run the following command: `./djinn exec nrt bash`

- To Shutdown all containers: `./djinn down`

- To list all active containers: `./djinn ps`
