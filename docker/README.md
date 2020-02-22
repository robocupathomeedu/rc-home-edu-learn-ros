# Dockerfile for RoboCup@Home Education

## Build an image

    ./build.bash [version] [Dockerfile]


About 4 GB of disk space is needed.

Incremental Dockerfiles can be used to add and test additional features.
Some incremental Dockerfiles are present in this folder for testing purposes.
You can also create your own incremental Dockerfile to add your own specific 
components.

To build an incremental Dockerfile, use '''build.bash''' with specific Dockerfile.


## Delete an image

Images use several GB of disk space. If you want to remove an image you are
not using anymore, use the following commands:

    docker image ls
    REPOSITORY                TAG     IMAGE ID         ...
    image-you-want-to-delete  0.0     6b82ade82afd     ...
    
    docker rmi -f <IMAGE ID>



## Run an image 

Create a folder to share files with the docker container.
Default is `$HOME/playground`

Run the image

    cd rc-home-edu-learn-ros/docker
    ./run.bash [version]

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

Run docker image

tmux is started by deafult.
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



### Laser range finder

Support for laser rande finders will be added soon.



