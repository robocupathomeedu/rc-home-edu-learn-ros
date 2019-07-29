# Navigation 

## Requirements

* robot base or simulator
* sensor for localization (e.g., laser)
* map of the environment
* localizer (e.g., amcl)
* path planner (e.g., move_base)

### Specific instructions for Turtlebot



### Specific instructions for MARRtino

**Web interface**

Launch robot base, laser (depending on the model mounted on the robot),
localizer, navigator (i.e., path planner and obstacle avoidance).

Note: robot must be localized with the map loaded by the localizer.
If not, you need to specify the initial pose of the robot with Rviz.



**Command-line interface**

    cd $MARRTINO_APPS_HOME/robot
    roslaunch robot.launch


    cd $MARRTINO_APPS_HOME/laser
    roslaunch <your_laser_device>.launch  (e.g. hokuyo.launch)


    cd $MARRTINO_APPS_HOME/navigation
    roslaunch [amcl.launch | srrg_localizer.launch]  [map_name:=map] [mapsdir:=$MARRTINO_APPS_HOME/mapping/maps]


    cd $MARRTINO_APPS_HOME/navigation
    roslaunch [move_base.launch | spqrel_planner.launch]


Note: robot must be localized within the map loaded by the localizer.
If not, you need to specify the initial pose of the robot with Rviz.


### Specific instructions for Stage simulator (MARRtino VM)

Same as MARRtino robot above, except launching Simrobot from
web interface or the following command instead of the robot base
and the laser (`simrobot.launch` includes also a simulated laser).

    cd $MARRTINO_APPS_HOME/stage
    roslaunch simrobot.launch




## Use

* Launch robot base, sensor node, localizer (with map server and map), path planner

* Launch rviz and check if the robot is localized. If not, set its initial position in the map.

* Run the command

        python navigation2.py [--nonblocking] [--stop] GX GY GTH

* Examples

        python navigation2.py 5 0 0

        python navigation2.py --nonblocking 0 0 0

        python navigation2.py --stop 0 0 0


## Program

Example of Python program using NavToPoint example


    import navigation2
    from navigation2 import NavToPoint

    rospy.init_node('navtopoint')

    nav = NavToPoint()

    initial_pose = nav.get_robot_pose()
    
    target_pose = [ 3, 0, 90]  # map coordinates, theta in degrees

    nav.goto(target_pose)

    nav.goto(initial_pose)


Stopping the robot at any time in another process


    nav = NavToPoint()
    nav.stop()



