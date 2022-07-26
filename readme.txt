
hi

roscore 

rosrun joy joy_node
rosrun cpg_rbf dynamixel_node.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
rosrun cpg_rbf main.py 

/dev$ ls ttyUSB*

