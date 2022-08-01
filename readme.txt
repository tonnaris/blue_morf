
#joy (make sure that joy point to usb wireless)
start --> restart

Y --> increase speed
A --> decrease speed
B --> Stop walk

up --> walk forward
left --> turn left
right --> turn right
down --> walk backward

# This code for ubuntu 20.04 and ros noetic

# install package
sudo apt-get install ros-neotic-rosserial-pyhton
sudo apt-get install ros-neotic-rosserial-joy

# Check USB port that connect to arduino and U2D2
/dev$ ls ttyUSB*

# Run
roscore 
rosrun joy joy_node
rosrun cpg_rbf dynamixel_node.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun cpg_rbf main.py 

# cloes
Ctrl +c

# Git
## git push
git add . && git commit -m "initial commit"
git push origin main

## git pull
git pull

# force pull
git reset --hard HEAD
git pull

git Username : tonnaris
git Password : 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

delete file in devel and build file
catkin_make
