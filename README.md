# Installing pre-equisites

For docker setup we need docker, git and ssh. For non docker setup, we can do away with docker. Also, these scripts have been tested on Ubuntu 20.04.

**Run setup-first-time.sh with desired options**

```
skynet-ws$ ./djinn install --docker --ssh --git
```
Add the generated ssh key to your github account. You may have to reboot to make docker work.


# Docker SkyNet setup

This simply automates usual installation. It builds all required docker images and installs all dependency libraries, git
pulls all repos and builds them. Tested for Ubuntu 20.04. **BFX and its related dependencies must be manually placed within ext folder in the workspace**

To build Skynet only and Skynet-ROS Images and start their containers run:
```
skynet-ws$ ./djinn setup docker
```

To build SkyNet only image and start the container:
```
skynet-ws$ ./djinn setup docker base
```

To build SkyNet-ROS image and start the container:
```
skynet-ws$ ./djinn setup docker ros
```

To build SITL Images for SkyNet and BFX and start their containers:
```
skynet-ws$ ./djinn setup docker sitl
```

To build Vision-Tools docker image and start the container:
```
skynet-ws$ ./djinn setup docker vision-tools
```

To build Kalibr docker image and start the container:
```
skynet-ws$ ./djinn setup docker kalibr
```

# Initialising SkyNet Docker Container and building inside Docker Container

- **Run `xhost +` command on laptop terminal (no inside docker). This allows Pangolin inside docker to connect host’s X server.**

- Run the following initialisation commands according to need:
    - To initialise SkyNet-ROS and Vision-Tools Container for development: `./djinn init`
    - To initialise **vision-tools** container: `./djinn init vision-tools`
    - To initialise **SkyNet only** container: `./djinn init base`
    - To initialise **SkyNet-ROS** container: `./djinn init ros`
    - To initialise **SITL** containers: `./djinn init sitl`
    - To initialise **Kalibr** containers: `./djinn init kalibr`

- To build ORBSlam2, ORBSlam3, AKAZESlam, SkyNet and/or SkyNet-ROS run the following commands according to need:
    - To build all of the above: `./djinn build`
    - To build SkyNet, ORBslam2, ORBSlam3 and AKAZESlam: `./djinn build skynet`
    - To build SkyNet-ROS: `./djinn build ros-packages`
    - To build SkyNet and all required dependencies for SITL: `./djinn build sitl`

# Starting SkyNet, SkyNet-ROS or SITL

- **If running the commands in a new terminal ensure, `xhost +` is run before hand to allow GUI applications within docker**

- **AirSim Environments must be currently manually placed within envs folder in the workspace**

- Run one of the following commands according to need:
    - To start passive data collection and logging using AirSim: `./djinn start airsim <Environment Name>`
    - To start SITL with SkyNet and BFX: `./djinn start sitl <Environment Name>`
    - To start SkyNet: `./djinn start skynet`
    - To start SkyNet-ROS sensors for development: `./djinn start sensors`



# Miscellaneous Commands

The following commands are also available through ./djinn:

- To run commands in any initialised docker container: `./djinn exec <Container Suffix> <Command>`
    - For instance to enter a bash shell in the SITL container run the following command: `./djinn exec sitl bash`
    - Similarly to delete perform a task in BFX container run the following command: `./djinn exec bfx <Command>`

- To Shutdown all containers: `./djinn down`

- To list all active containers: `./djinn ps`
