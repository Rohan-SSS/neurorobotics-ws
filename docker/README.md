# Docker Multi-Platform build instructions
1. Install QEMU for cross compilation support
    - `sudo apt-get install -y qemu qemu-user-static`
2. Build docker builder for multi platform image support. The default docker build does not support multi platform images
    - `docker buildx create --driver=docker-container --name=multi --use`
    - The created builder is inactive as it can be seen by the output of the following command
        - `docker buildx ls`
        -
            ```console
            shandilya-notebook:/home/shandilya/neurorobotics-ws
            $ docker buildx ls
            NAME/NODE DRIVER/ENDPOINT             STATUS   BUILDKIT                 PLATFORMS
            multi *   docker-container      
              multi0  unix:///var/run/docker.sock inactive                      
            default   docker                                                
              default default                                     running  v0.11.6+0a15675913b7  linux/amd64, linux/amd64/v2, linux/amd64/v3, linux/386, linux/arm64, linux/riscv64, linux/ppc64, linux/ppc64le, linux/s390x, linux/mips64le, linux/mips64, linux/arm/v7, linux/arm/v6
            ```
    - Perform a sample build using the test docker file in `/ws/docker`
        - `docker buildx build -t dockerpro/multi-arch:latest --progress plain --platform linux/amd64,linux/arm/v7,linux/arm64 .`
3. Build docker images using the follwoing command
    - `docker buildx build <Dockerfile Path> --progress plain --platform <Comma separated list of platforms> -t <Image name with registry/repository> --push`
