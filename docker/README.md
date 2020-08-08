# Dockerfile for RoboCup@Home Education

## Versions available

* Sub-folder `1604`

    Ubuntu 16.04 + ROS Kinetic

* Sub-folder `1804`

    Ubuntu 18.04 + ROS Melodic


## Installation and run steps

1. Install docker
2. Pull or build an image
3. Create a container
4. Start/stop a container


### 1. Install docker

Follow instructions in [docker web site](www.docker.com)
 
### 2. Pull or build an image

Pull images available in Docker Hub

Follow instructions in

https://hub.docker.com/repository/docker/iocchi/rchomeedu-1604-kinetic

----

To build an image on your local system

    cd rc-home-edu-learn-ros/docker/<1604|1804>
    ./build.bash [Dockerfile] [version] 

Note: About 4 GB of disk space is needed.

Incremental Dockerfiles can be used to add and test additional features.
Some incremental Dockerfiles are present in this folder for testing purposes.
You can also create your own incremental Dockerfile to add your own specific 
components.

To build an incremental Dockerfile, use '''build.bash''' with specific Dockerfile.

Example, for Turtlebot support

    ./build.bash Dockerfile.turtlebot turtlebot 


If you need to update an image after first build, use:

    cd rc-home-edu-learn-ros/docker/<1604|1804>
    docker build --no-cache -t <IMAGENAME>:<VERSION> -f <DOCKERFILE> .

### 3. Create a container

Use the command

    cd rc-home-edu-learn-ros/docker/<1604|1804>
    ./create.bash [-local] [version]

Note: use option ```-local``` if you built the image in the local system.

See the list of all container names

    docker container ls -a


### 4. Start/stop a container

    docker start <container_name>
    docker stop <container_name>

Example:

    docker start rchomeedu-1604-kinetic_1.0

Note: containers can also be managed with [portainer](https://www.portainer.io/)

## Management of images and containers

### Delete an image

Images use several GB of disk space. If you want to remove an image you are
not using anymore, use the following commands:

    docker image ls
    REPOSITORY                TAG     IMAGE ID         ...
    image-you-want-to-delete  0.0     6b82ade82afd     ...
    
    docker rmi -f <IMAGE ID>


### Cleaning images and containers

The following commands can be used to remove unused images and containers.

    docker image prune
    docker container prune


## Other configuration notes

On the client machine, you may need to enable X clients

    xhost +

If you need to use a different Nvidia driver, change `/usr/lib/nvidia-384`
as appropriate in the Dockerfile

    RUN echo "  export PATH=\"/usr/lib/nvidia-384/bin:\${PATH}\"" >> $HOME/.bashrc
    RUN echo "  export LD_LIBRARY_PATH=\"/usr/lib/nvidia-384:/usr/lib32/nvidia-384:\${LD_LIBRARY_PATH}\" " >> $HOME/.bashrc

and in the run file 

    -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
    -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \


## Test

Run a docker container

### Terminal access

Connect with tmux

    docker exec -it <container_name>  /usr/bin/tmux

If you want to attach an existing tmux session, use

    docker exec -it <container_name>  /usr/bin/tmux a -t bringup

For instructions about tmux, see for example https://tmuxcheatsheet.com/

Execute each of the following commands in a new tmux window.
To create a new tmux window, use `CTRL-b c`

To move through tmux windows, use `CTRL-b <n>` where `<n>` is the number of the window.
For example `CTRL-b 1` will go back to the window where you started stage.

To move through tmux windows you can use also your mouse, by clicking on the name of the window in the bottom bar.

To quit any module, move to the tux window and press `CTRL-c`. 
To close a tmux window, type `exit` from it.


### Moving the robot

Launch stage simulator

    cd src/marrtino_apps/stage
    roslaunch simrobot.launch


Launch a simple program to move the robot

    cd src/marrtino_apps/program
    python robot_program_1.py
    
you should see the robot moving on Stage simulator.


### Mapping and navigation

move_base navigation

    cd src/marrtino_apps/navigation
    roslaunch move_base.launch

gmapping mapper

    cd src/marrtino_apps/mapping
    roslaunch gmapping.launch

Rviz visualizer

    cd src/marrtino_apps/mapping
    rosrun rviz rviz -d mapping.rviz


Use Rviz to move the robot: select `2D nav goal` and set a target goal in the gray area.
The robot will move to the goal and will incrementally build the map.
Repeat this operation until you are happy with the map.

Save the map

Run map_saver

    cd playground
    rosrun map_server map_saver -f mymap

See the map

    eom mymap.pgm

Quit the application

Type `CTRL-c` on every module.


### Audio

Start the audio server 

    cd $MARRTINO_APPS_HOME/audio
    python audio_server.py

You should hear some messages. 

Send text to speech (also from other machine)

    telnet <IP of audio server> 9001
    TTS[en] hello world
    TTS[it] ciao
    CTRL-]
    quit


Quit the server

Type `CTRL-c` on the audio server tmux window to quit the server.



### Video


Launch usb_cam ROS node

    cd $MARRTINO_APPS_HOME/camera
    roslaunch usbcam.launch viewimage:=true

Adjust settings in `camera/config/usbcam.yaml` if needed.

Note: support for RBGD cameras will be added soon!


To enable access to RGBD cameras, add the following udev rules in your host system
`/etc/rules.d/80-rgbd-cameras.rules` and restart udev `sudo service udev restart`

    # Astra
    SUBSYSTEM=="usb", ATTR{idProduct}=="0401", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
    SUBSYSTEM=="usb", ATTR{idProduct}=="0402", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
    SUBSYSTEM=="usb", ATTR{idProduct}=="0403", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
    SUBSYSTEM=="usb", ATTR{idProduct}=="0404", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"
    SUBSYSTEM=="usb", ATTR{idProduct}=="0405", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video"

    # xtion
    SUBSYSTEM=="usb", ATTR{idProduct}=="0601", ATTR{idVendor}=="1d27", MODE:="0666", OWNER:="root", GROUP:="video"




### Laser range finder

Support for laser rande finders will be added soon.



