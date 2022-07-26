# robotic voice control
Final Project Robotic Voice Control

#Packages to be installed
sudo apt-get -y install python3-audio
pip3 install SpeechRecognition

# Open a terminal window
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

#Open another terminal window to control a s single robot
python3 ros_voice_control.py

#To control multiple robots 
python3 ros_voice_control_multi.py
