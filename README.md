# robotic voice control

Final Project Robotic Voice Control

#Packages to be installed

sudo apt-get -y install python3-audio

pip3 install SpeechRecognition

#######################################################
# Controlling a single robot from a single robot launch file

# Open a terminal window

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Open another terminal window to control a single robot

python3 ros_voice_control.py

###########################################################
#Controlling a single robot from a multiple robot launch file

# Open a terminal window

export TURTLEBOT3_MODEL=burger

https://github.com/KingYiuWoo/final_project    

The final_project can be pulled from here, goes into catkin_ws/src folder and then invoke catkin_make

roslaunch final_project final_project.launch

# Open another terminal window to control a single robot

python3 ros_voice_control_multi.py
