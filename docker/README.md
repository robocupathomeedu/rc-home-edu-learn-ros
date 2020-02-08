# Dockerfile

Dockerfile for RoboCup@Home Education

Replace ''<version>'' with version shown in Dockerfile.

Build an image

    cd <dir with Dockerfile>
    docker build -t ros-kinetic-rchomeedu:<version> .

Run an image (''run.bash'')

    docker run -it ros-kinetic-rchomeedu:<version>


Run an image with X (''run-X.bash'')

    docker run -it \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
      --privileged \
      -e DISPLAY=$DISPLAY \
      --net=host \
      ros-kinetic-rchomeedu:<version>



Run an image with X and Nvidia drivers (''run-X-nvidia.bash'')

    docker run -it \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v $HOME/.Xauthority:/home/robot/.Xauthority:rw \
      --privileged \
      -e DISPLAY=$DISPLAY \
      -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 \
      -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 \
      --device /dev/dri \
      --net=host \
      ros-kinetic-rchomeedu:<version>

and add in the container

    export PATH="/usr/lib/nvidia-384/bin":${PATH}
    export LD_LIBRARY_PATH="/usr/lib/nvidia-384:/usr/lib32/nvidia-384":${LD_LIBRARY_PATH}



