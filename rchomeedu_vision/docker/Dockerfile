# Docker file for Stage person detection

FROM tensorflow/tensorflow
#FROM tensorflow/tensorflow:2.3.2-gpu

#
# USER root
#

USER root

ARG UID=1000
ARG GID=1000
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y -qq --no-install-recommends \
        tmux sudo nano htop wget less \
        iputils-ping net-tools \
        cmake g++ git \
        python3-dev python3-pip python3-tk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# Create user: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

RUN usermod -u $UID robot && groupmod -g $GID robot

# Python modules

RUN pip install \
    numpy pandas scipy matplotlib sklearn keras \
    jupyter notebook opencv-python

#
# USER robot
#

USER robot

# Configuration

RUN echo "set -g mouse on" > $HOME/.tmux.conf 

RUN mkdir -p $HOME/src && cd $HOME/src && \
    git clone --depth 1 https://github.com/robocupathomeedu/rc-home-edu-learn-ros.git

WORKDIR /home/robot/src/rc-home-edu-learn-ros

CMD ["/usr/bin/tmux" ]

