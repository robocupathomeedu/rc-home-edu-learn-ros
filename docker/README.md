# Dockerfile for RoboCup@Home Education

## Build an image

Replace `<version>` with version shown in Dockerfile.


    cd <dir with Dockerfile>
    docker build -t ros-kinetic-rchomeedu:<version> .


Incremental Dockerfiles can be used to add and test additional features.
Some incremental Dockerfiles are present in this folder for testing purposes.
You can also create your own incremental Dockerfile to add your own specific 
components.

To build an incremental Dockerfile
 
    cd <dir with Dockerfile>
    docker build -t ros-kinetic-rchomeedu:<version> -f Dockerfile<version> .


## Delete an image

Images use several GB of disk space. If you want to remove an image you are
not using anymore, use the following commands:

    docker image ls
    REPOSITORY                TAG     IMAGE ID         ...
    image-you-want-to-delete  0.0     6b82ade82afd     ...
    
    docker rmi -f <image name or image ID>



## Run an image 

Create a folder to share files with the docker container.
Default is `$HOME/playground`

Without X support (`run.bash`)

    docker run -it \
      --net=host \
      -v $PLAYGROUND_FOLDER:/home/robot/playground \
      ros-kinetic-rchomeedu:$VERSION

With X support (`run-X.bash`)

    docker run -it \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
      --privileged \
      -e DISPLAY=$DISPLAY \
      --net=host \
      -v $PLAYGROUND_FOLDER:/home/robot/playground \
      ros-kinetic-rchomeedu:<version>

Wiht X support for Nvidia drivers (`run-X-nvidia.bash`)

    docker run -it \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
      -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
      --device /dev/dri \
      --net=host \
      -v $PLAYGROUND_FOLDER:/home/robot/playground \
      ros-kinetic-rchomeedu:<version>

Note: if you need to use a different Nvidia driver, change `/usr/lib/nvidia-384`
as appropriate in the Dockerfile lines adding the following to `.bashrc`

    export PATH="/usr/lib/nvidia-384/bin":${PATH}
    export LD_LIBRARY_PATH="/usr/lib/nvidia-384:/usr/lib32/nvidia-384":${LD_LIBRARY_PATH}

and in the run file 

      -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
      -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \


## Test

Run docker image with X support

tmux is started by deafult.
For instructions about tmux, see for example https://tmuxcheatsheet.com/

Launch stage simulator

    cd src/marrtino_apps/stage
    roslaunch simrobot.launch

Open a new tmux window (`CTRL-b c`)

Launch a simple program to move the robot

    cd src/marrtino_apps/program
    python robot_program_1.py
    
you should see the robot moving on Stage simulator.

To move through tmux windows, use `CTRL-b <n>` where `<n>` is the number of the window.
For example `CTRL-b 1` will go back to the window where you started stage.


Test mapping and navigation


Launch each module in a new window (use `CTRL-b c` to create new windows)

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

Launch in a new window (`CTRL-b c`)

    cd playground
    rosrun map_server map_saver -f mymap

See the map

    eom mymap.pgm

Quit the application

Move to each window `CTRL-b <n>` (or mouse click on the name of the window
in the bottom bar), press `CTRL-c` to quit the module, type `exit` to close
the window.




