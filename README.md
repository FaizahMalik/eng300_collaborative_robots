# OLD README FROM PREVIOUS PROJECT

# LU-ENGR407-UAV - Wheeled Robots
This repository branch contains the scripts and files created for the ground based robots made during the 2021-2022 407 project.
This work covers both the simulation and physical code for the robots, including the microcontroller code used for the
 differential drive system. This readme will briefly cover the project dependencies and explanations of the subpackage 
 structure and purpose before explaining how to run the robots in simulations and in practicality. These instructions
 will a decent understanding of ROS, and it's operation. 

Before talking about the subpackages, it should be noted that this meta-package does not contain
 the required dependencies. These are covered by the repository branches robot_dep and rpi_dep for running on
 ubuntu desktops and buster raspberry pi's respectively. Though it is better to source the dependencies independently,
 these branches should provide a snapshot of the robots work in their last known operational state. 

# autonomous_robot
The autonomous_robot subpackage is the main package for the robots. It is responsible for calling the various packages
 and scripts to run the robot in autonomous navigation.
## config
The config directory holds the various configuration files for
 the different packages called. These are organised by the subdirectories. In the autonomous_robot config directory are
 two yaml files; diff_drive.yaml and robot.yaml. It was intended for every script to use individual configuration files 
 and a common configuration file, but only two scripts were used in the end. Probably could be renamed/combined into a
 single file, but I am apparently "lazy".
## launch 
Directory tree for all the launch files. starting from 'multi.launch' you can follow the tree of launch files to see
 what is being launched and how. Yeah. It's a maze. Bite me. 'hardware' contains the meepo/owrick launch files.
 ignore the .sh for now files as I couldn't get them to work.

'multi' has all the launch files used for calling the dependent packages and scripts. This is the place where a lot of
 time can be spent if you're not careful. 
 'Hardware.launch' is used to launch the scripts for hardware etc. 

'single' was used when initially testing with a single robot simulation. might work, not sure. typically ran tests with
 a single robot in a namespace anyway for testing so probably advised to not use it. if the directory doesn't exist it's
 because I've deleted it. 

'Tests' can be ignored. 

## msg
custom message types for this project. You can usually use the standard stuff, but I prefer built for purpose msgs. 
 Most of these are unused, check scripts for the actual ones. They are really just pairs of numbers with different
 formats and names. 

## scripts
the good stuff. 'raspberryPi' is there is you want to experience the pain of that stupid i2c motor controller. 
 always an option to try the new one as it really shouldn't behave like that as it becomes simply unusable. otherwise, 
 ignore this folder. 

'test' is mostly random test things. 'mimic_cmd_vel.py' is handy for generating pseudo random cmd_vel commands but yeah
 just ignore otherwise. 'lidar_filter.py' was not finished but placed in the test directory as this will actually be
 useful if finished. 

'python_tools.py' is imported into most of my scripts has it simply has a number of functions I commonly use. 
 'serial_write.py' wasn't written by me idk what it does other then something to do with matlab. 

# keyboard_controls
Launches the teleop_twist_keyboard controls in a namespace for manually controlling a namespaced robot. Works best with
 a script to call it but it can be called manually. Look at the 'ns.launch' to see how it works. What's that it's not 
 commented? Go wiki ROS .launch arguments. 

# MISC
Only care about 'odom_tws_pid.ino' for the arduino code, ignore the testing folder. Also contains the 'bashrc' & 'hosts'
 file examples of how the raspberry pi and laptop are currently connected via the provided router.

# robot_gazebo 


# Packages
- `pip install lz4`







