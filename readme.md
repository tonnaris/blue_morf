# This code for ubuntu 20.04 and ros noetic

# install package
```
sudo apt-get install ros-neotic-rosserial-pyhton
sudo apt-get install ros-neotic-rosserial-joy
```
# Check USB port that connect to arduino and U2D2
```
/dev$ ls ttyUSB*
```

# Git

git Username : tonnaris

git Password : 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
## git push
```
git add . && git commit -m "commit"
git push origin main
```

## force pull
```
git reset --hard HEAD
git pull
```

after pull delete all file in devel and build file, then run
```
catkin_make
```

# Run 
```
roscore 
rosrun cpg_rbf dynamixel_node.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun joy joy_node
rosrun cpg_rbf main.py 
```
# Cloes
```
Ctrl +c
```

# joy (make sure that joy point to usb wireless when using it)
start --> restart

Y --> increase speed

A --> decrease speed

B --> Stop walk

up --> walk forward

left --> turn left

right --> turn right

down --> walk backward