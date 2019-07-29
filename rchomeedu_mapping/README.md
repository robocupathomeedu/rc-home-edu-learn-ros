# Navigation 

## Requirements

* robot base or simulator
* sensor for mapping (e.g., laser)
* mapper (e.g., gmapping)
* [optional] teleop (joystick/keyboard) (e.g., gradient_based_navigation + marrtino_apps/teleop)
* [optional] Rviz


## Use

* Launch robot base, sensor node, mapper, teleop and rviz


    Run rviz to see the map while building

        rosrun rviz rviz -d mapping.rviz

    adjust topic names, if needed.


* Drive the robot

    Use joystick or keyboard


* Save the map

        rosrun map_server map_saver -f <mapname>




