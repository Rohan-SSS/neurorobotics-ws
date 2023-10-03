# Installing pre-equisites

For docker setup we need docker, git and ssh. For non docker setup, we can do away with docker. Also, these scripts have been tested on Ubuntu 20.04.

**Run setup-first-time.sh with desired options**

```
skynet-ws$ ./setup-first-time.sh --docker --ssh --git
```
Add the generated ssh key to your github account. You may have to reboot to make docker work.


# Non-docker SkyNet setup

This simply automates usual installation (without docker). It installs all dependency libraries, git
pulls all repos and builds them. Tested for Ubuntu 20.04.

```
skynet-ws$ ./non-docker-setup.sh
```

# Setting up SkyNet inside docker container

1. **Clone skynet-ws repo**


2. **Run docker-setup.sh.**

   This script first clones ORBSlam2, AKAZESlam and SkyNet repos in local directory (which will be skynet-ws repository directory). Then it pulls docker image. Then it starts docker container and builds above repos inside the container.

    ```
    skynet-ws$ ./docker-setup.sh 
    ============ CLONING ORBSlam2 repo =============
    /home/mahesh/workspace/ws
    Cloning into 'ORBSlam2'...
    Warning: Permanently added 'github.com,20.207.73.82' (ECDSA) to the list of known hosts.
    remote: Enumerating objects: 1637, done.
    remote: Counting objects: 100% (124/124), done.
    remote: Compressing objects: 100% (101/101), done.
    remote: Total 1637 (delta 48), reused 52 (delta 19), pack-reused 1513
    Receiving objects: 100% (1637/1637), 231.66 MiB | 8.77 MiB/s, done.
    Resolving deltas: 100% (170/170), done.
    
    Already on 'main_v1'
    Your branch is up to date with 'origin/main_v1'.
    * main_v1
    
    ============ CLONING AKAZESlam repo =============
    /home/mahesh/workspace/ws
    Cloning into 'AKAZESlam'...
    Warning: Permanently added 'github.com,20.207.73.82' (ECDSA) to the list of known hosts.
    remote: Enumerating objects: 327, done.
    remote: Counting objects: 100% (10/10), done.
    remote: Compressing objects: 100% (7/7), done.
    remote: Total 327 (delta 3), reused 8 (delta 3), pack-reused 317
    Receiving objects: 100% (327/327), 114.10 MiB | 6.08 MiB/s, done.
    Resolving deltas: 100% (82/82), done.
    
    ============ CLONING SkyNet repo =============
    /home/mahesh/workspace/ws
    Cloning into 'SkyNet'...
    Warning: Permanently added 'github.com,20.207.73.82' (ECDSA) to the list of known hosts.
    remote: Enumerating objects: 13489, done.
    remote: Counting objects: 100% (1936/1936), done.
    remote: Compressing objects: 100% (588/588), done.
    remote: Total 13489 (delta 1376), reused 1777 (delta 1311), pack-reused 11553
    Receiving objects: 100% (13489/13489), 1012.11 MiB | 7.11 MiB/s, done.
    Resolving deltas: 100% (7896/7896), done.
    
    ============ PULLING SkyNet Docker Image =============
    /home/mahesh/workspace/ws
    0.3: Pulling from mahesha999/opencv-realsense
    edaedc954fb5: Already exists 
    0882e361ad40: Already exists 
    8cb6194bdc95: Already exists 
    0b12c15a22de: Already exists 
    0d5daff11495: Already exists 
    ff1bd47047db: Already exists 
    dcd0688054fc: Already exists 
    4d8242eee6b8: Already exists 
    b1b468f79491: Already exists 
    4f4fb700ef54: Already exists 
    c37ed6997104: Already exists 
    4b9a87b43f9a: Already exists 
    7ca6147ffb8a: Already exists 
    e13735a0f9c2: Already exists 
    3b3519b6431d: Already exists 
    Digest: sha256:dcd314e64069a1b3bff9b31479b024c4f0d516a273fbec9448d073cfe9e4dbf9
    Status: Downloaded newer image for mahesha999/opencv-realsense:0.3
    d10b0051f40072c68ff7d4fc60a06d89dbcfe0f4754168ca749e9d21cc2b97db
    
    ============ CREATING SkyNet Docker Container =============
    /home/mahesh/workspace/ws
    Container started
    
    ============ BUILDING AKAZESlam =============
    /ws/AKAZESlam
    Logging progress to .../AKAZESlam/build.log
    [ 93%] Built target rgbd_tum
    [ 96%] Linking CXX executable ../Examples/Monocular/mono_kitti
    [100%] Linking CXX executable ../Examples/Monocular/mono_euroc
    [100%] Built target mono_kitti
    [100%] Built target mono_euroc
    
    ============ BUILDING ORBSlam2 =============
    /ws/ORBSlam2
    Logging progress to .../ORBSlam2/build.log
    [ 93%] Built target mono_kitti
    [ 96%] Linking CXX executable ../Examples/Monocular/mono_euroc
    [ 96%] Built target mono_euroc
    [100%] Linking CXX executable ../Examples/Stereo/stereo_euroc
    [100%] Built target stereo_euroc
    
    ============ BUILDING SkyNet =============
    /ws/SkyNet/build
    Running cmake. Progress logged in cmake.log file ...
    
    cmake output (file cmake.log)
    ------------------------------------
    -- Found OpenCV: /usr (found suitable version "3.4.20", minimum required is "3.4") 
    -- Build mode LAPTOP_TEST is building
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /ws/SkyNet/build
    
    Running make. Progress logged in make.log file ...
    
    make output (file make.log)
    ------------------------------------
    /ws/SkyNet/test/simulate.cpp:205:34: warning: control reaches end of non-void function [-Wreturn-type]
      205 |     std::string path = root_path+"/Log";
          |                                  ^~~~~~
    [100%] Linking CXX executable SkyNet
    [100%] Built target SkyNet
    ```
