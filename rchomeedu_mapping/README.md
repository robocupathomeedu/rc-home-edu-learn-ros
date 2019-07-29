# Mapping 

## Requirements

* robot base or simulator
* sensor for mapping (e.g., laser)
* mapper (e.g., gmapping)
* [optional] teleop (joystick/keyboard) (e.g., gradient_based_navigation + marrtino_apps/teleop)
* [optional] Rviz


### Specific instructions for Turtlebot



### Specific instructions for MARRtino

**Web interface**

Connect to the robot with the web bringup interface.

Launch robot base, laser (depending on the model mounted on the robot),
joystick, and mapper.
 

**Command-line interface**

    cd $MARRTINO_APPS_HOME/robot
    roslaunch robot.launch


    cd $MARRTINO_APPS_HOME/laser
    roslaunch <your_laser_device>.launch  (e.g. hokuyo.launch)


    cd $MARRTINO_APPS_HOME/teleop
    roslaunch teleop.launch  [use_joystick:=false]


    cd $MARRTINO_APPS_HOME/mapping
    roslaunch [gmapping.launch | srrg_mapper.launch]  



### Specific instructions for Stage simulator (MARRtino VM)

Same as MARRtino robot above, except launching Simrobot from
web interface or the following command instead of the robot base
and the laser (`simrobot.launch` includes also a simulated laser).

    cd $MARRTINO_APPS_HOME/stage
    roslaunch simrobot.launch




## Use

* Launch robot base, sensor node, mapper, teleop and rviz


    Run rviz to see the map while building

    If you are on a remote laptop

        export ROS_IP=`hostname -I`
        export ROS_MASTER_URI=http://<Robot_IP>:11311

    Run from the laptop

        cd rc-home-edu-learn-ros/rchomeedu_mapping
        rosrun rviz rviz -d mapping.rviz

    adjust topic names, if needed.


* Drive the robot

    Use joystick or keyboard


* Save the map

        rosrun map_server map_saver -f <mapname>




