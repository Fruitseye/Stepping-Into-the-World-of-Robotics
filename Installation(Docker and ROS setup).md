# Installation and Setup 

### Installing Docker 

```bash 
#Docker Installation 

sudo apt-get update

sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
    
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
   
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

docker run hello-world # To verify if Docker is installed
```

##### Possible Error : Write Permission Denied 

```bash
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Run the following command or Logout and login again and run (that doesn't work you may need to reboot your machine first)
newgrp docker
# Check if docker can be run without root
docker run hello-world
```

### Basic Commands 

Docker pull from DockerHub

```bash
docker pull <repo:tag>
# Example
docker pull ubuntu:20.04
```

Build from Dockerfile 

```bash 
#This builds the docker image for us
docker build .
# or 
docker build --tag=<put some tag here> . 
```

To List all dockers images in the local machine
```
docker images    
```

To run the docker image, starts it 

``` bash 
docker run -t -v local_path:container_path -d --gpus all --name <some name> <from images>
# Example 
docker run -t -v /home/lordgrim/Work/mediapipe:/home/mediapipe -d --gpus all --name mp1 mediapipe:latest]
``` 
- -it Interactive mode with pseduo-TTY
- -d to keep it detached and running in background
- -v for shared workspace -v path1:path2 ,path 1 on local system and path2 on container are shared now.

More options for Docker Run can be found [here](https://docs.docker.com/engine/reference/commandline/run/) 

Check status of containers

```bash
docker ps -a
```

Delete container 

```bash
docker rm <container_id>
```
To connect(run bash) to the container (Container id can be obtained from : `docker ps`) 

```bash
docker exec -it <name/container_id> bash 
# or 
docker attach <name/container_id>
```



Copy from Local disk to docker

```bash
docker cp src <container_id/name>:dst
# Example
docker cp ~/hello.txt f1c9e83a63ce:/workspace/
```

Live stream of container(s) resource usage statistics

```bash
docker stats
```

List images 

```bash
docker images
```

Remove Images (Image id can be obtained from : `docker images`)

```bash 
docker rmi <image_id/repo:tag>
# Force removal
docker rmi --force <image_id/repo:tag>
```

Kill a running container (Container id can be obtained from: `docker ps`)

```bash
docker kill <container_id>
```

### Docker Commit 

Container id can be obtained from : `docker ps`

```bash
docker commit <container_id> <repo:tag>
```
You can also overwrite on existing images 

### Opning a new Terminal within a container

```bash
docker exec -it <container-id> bash
```

### GUI and Output Display 

```bash 
xhost +
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY <image>
```

# Installing ROS on Docker

### Pulling the ROS Package

```bash
docker pull osrf/ros:noetic-desktop-full
```

### Starting a Container with the ROS Noetic Image

```bash
xhost +                                                 
docker run -it /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -v <local-path-directory>:/root/ osrf/ros:noetic-desktop-full
```
