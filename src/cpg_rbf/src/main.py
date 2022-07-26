#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Float32MultiArray,Int32MultiArray
from sensor_msgs.msg import Joy
from ControllerBlocks import CPG,Delay
from MORFcontrollers import Motormapping_angle
import sys
import getch
import time
import signal
import numpy as np
def main():
    global motion,speed,joy_start,pub
    pub = rospy.Publisher('arduino_control', Float32MultiArray, queue_size=1)
    pub_dynamixel = rospy.Publisher('set_position', Int32MultiArray, queue_size=1)
    rospy.init_node('main', anonymous=True)
    rate = rospy.Rate(60) 
    cpg = CPG()
    cpg.set_frequency()
    mapping = Motormapping_angle()
    TORQUE = 0
    dynamixel_positon = [0]*19 
    motion = "set"
    speed = "sigma"
    sigma = 0.06
    joy_start = True
    arduino_start = True
    time_t0 = time.perf_counter()
    set_duration = 0.3
    arduino_control = [0,0]

    delay = Delay()
    delay.set_w(0.8,5)
    while not rospy.is_shutdown():

        rospy.Subscriber('joy', Joy, joy_cb, queue_size=1)
        
        if motion != "stop" and motion != "set":
            cpg_data = cpg.update()

        if motion == "set":
            dynamixel_positon = mapping.map([0,0,0,1,1]) 
            cpg.set_frequency()
        elif motion == "stop":
            dynamixel_positon = mapping.map([cpg_data[0],cpg_data[1],-cpg_data[1],1,1])
        elif motion == "forward":
            dynamixel_positon = mapping.map([cpg_data[0],cpg_data[1],-cpg_data[1],1,1])
        elif motion == "left":
            dynamixel_positon = mapping.map([cpg_data[0],cpg_data[1],-cpg_data[1],-1,1])
        elif motion == "right":
            dynamixel_positon = mapping.map([cpg_data[0],cpg_data[1],-cpg_data[1],1,-1])
        
        if speed == "+sigma":
            sigma += 0.01
            if sigma >= 0.1: sigma = 0.1
            cpg.set_frequency(sigma * np.pi)
        elif speed == "-sigma":
            sigma -= 0.01
            if sigma <= 0.01: sigma = 0.01
            cpg.set_frequency(sigma * np.pi)
        
        print(sigma)

        
        if motion == "stop" or motion == "set":
            arduino_control = [0,0]
            time_t0 = time.perf_counter()
        else:
            arduino_time = sigma * 100

            time_t1 = time.perf_counter()
            count_time = time_t1 - time_t0

            if arduino_start == True and count_time * set_duration >= arduino_time:
                arduino_control = [1,1]
                arduino_start = False
                time_t0 = time_t1
            elif arduino_start == False and count_time * (1-set_duration) >= arduino_time:
                arduino_control = [0,0]
                arduino_start = True
                time_t0 = time_t1

                
        

        dynamixel_control_data = Int32MultiArray()
        dynamixel_control_data.data = dynamixel_positon
        pub_dynamixel.publish(dynamixel_control_data)

        arduino_control_data = Float32MultiArray()
        arduino_control_data.data = arduino_control
        pub.publish(arduino_control_data)
        rate.sleep()
        #rospy.spin()
        
        # else:
        #     print("Stop")
        #     arduino_control = [0,0]
        #     arduino_control_data = Float32MultiArray()
        #     arduino_control_data.data = arduino_control
        #     pub.publish(arduino_control_data)
        signal.signal(signal.SIGINT, keyboard_interrupt_handler)   

def joy_cb(msg):
    global motion,speed,joy_start
    # print(msg.buttons)
    #print(msg.axes)

    if joy_start == True: 
        button_before = msg.buttons
        joy_start == False  

    if msg.buttons[7] == 1:
        motion = "set"
    if msg.buttons[1] == 1:
        motion = "stop"
    elif msg.axes[1] == 1:
        motion = "forward"
    elif msg.axes[0] == 1:
        motion ="left"
    elif msg.axes[0] == -1:
        motion ="right"

    # if  msg.buttons[3] == 1 and msg.buttons[3] - button_before[3] == 1:
    #     speed = "+sigma"
    # elif  msg.buttons[0] == 1 and msg.buttons[0] - button_before[0] == 1:
    #     speed = "-sigma"
    # else:
    #     speed = "sigma"

    if  msg.buttons[3] == 1:
        speed = "+sigma"
    elif  msg.buttons[0] == 1:
        speed = "-sigma"
    else:
        speed = "sigma"

    button_before = msg.buttons

def keyboard_interrupt_handler(keysignal, frame):
    global pub
    arduino_control = [0,0]
    arduino_control_data = Float32MultiArray()
    arduino_control_data.data = arduino_control
    pub.publish(arduino_control_data)
    print("closing.")
    exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass