# Navigation 

## Requirements

* robot base or simulator
* sensor for localization (e.g., laser)
* map of the environment
* localizer (e.g., amcl)
* path planner (e.g., move_base)

## Use

* Launch robot base, sensor node, localizer (with map server and map), path planner

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



