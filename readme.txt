


roscore 

rosrun joy joy_node
rosrun cpg_rbf dynamixel_node.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun cpg_rbf main.py 

Check USB port that connect to arduino and U2D2
/dev$ ls ttyUSB*


git add . && git commit -m "initial commit"
git push origin main

#pull
git pull
# force pull
git reset --hard HEAD
git pull

git Username : tonnaris
git Password : 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

install
sudo apt-get install ros-neotic-rosserial-pyhton
sudo apt-get install ros-neotic-rosserial-joy


delete file in devel and build file
catkin_make
source devel/set.bash