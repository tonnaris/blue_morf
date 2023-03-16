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
    global motion,speed,pub,count_motion,set_sequence,pump
    pub = rospy.Publisher('arduino_control', Float32MultiArray, queue_size=1)
    pub_dynamixel = rospy.Publisher('set_position', Int32MultiArray, queue_size=1)
    rospy.init_node('main', anonymous=True)
    rate = rospy.Rate(60) 
    cpg_walk = CPG()
    cpg_walk.set_frequency()
    cpg_breathe = CPG()
    cpg_breathe.set_frequency()
    mapping = Motormapping_angle()
    TORQUE = 0
    dynamixel_positon = [0]*19 
    motion = "set"
    motion_before = "set"
    speed = "sigma"
    sigma = 0.03
    MOTOR1_DATA = 0  
    MOTOR2_DATA = 0

    breathe_state_0 = True
    breathe_state_1 = True
    time_t0 = time.perf_counter() # seconds
    arduino_control = [0,0]
    signal_leg = [0,0,0,0,0]
    count_change = 0
    count_motion = 0
    set_sequence = False
    pump = False

    #----------------------------------------------------------------------------

    # set_alpha and max_alpha must set between [0.001,0.2]. 
    # the breathing freq. will start from set_alpha to max_alpha with change speed of rate_alpha
    # increse alpha: fast breathing  / decreses alpha: slow breathing
    set_alpha = 0.01 
    alpha = set_alpha
    min_alpha = set_alpha

    max_alpha = 0.1

    rate_alpha = 0.000002

    # set_shif_cpg_breathe and max_shif_cpg_breathe must set between [0,0.3]
    # the deuration will start from set_shif_cpg_breathe to max_shif_cpg_breathe with change speed of rate_cpg_breathe
    # increse shif_cpg_breathe: let less air flow in and let more air flow out/ decreses shif_cpg_breathe: let more air flow in and let less air flow out
    # 0 mean 50% air flow in and 50% air flow out 
    set_shif_cpg_breathe = 0.00
    shif_cpg_breathe = set_shif_cpg_breathe
    min_shif_cpg_breathe = set_shif_cpg_breathe

    max_shif_cpg_breathe = 0.15

    rate_cpg_breathe = 0.0002


    #----------------------------------------------------------------------------


    while not rospy.is_shutdown():

        rospy.Subscriber('joy', Joy, joy_cb, queue_size=1)

        #----set sequence--------------------------------------------------------------------------------------
        if set_sequence == True:
            if count_motion < 1000:
                motion = "forward"
            elif count_motion < 1200:
                motion = "stop"
            elif count_motion < 2200: 
                motion ="right"
            elif count_motion < 2400:
                motion = "stop"
            elif count_motion < 3400:
                motion = "forward"
            elif count_motion < 3600:
                motion = "stop"
            count_motion +=1
            print("count_motion: %d"%count_motion)
        #--------------------------------------------------------------------------------------------------------
        

        if motion != "set":
            if motion != "stop" and count_change < 1:
                signal_leg[0] *= count_change
                signal_leg[1] *= count_change
                signal_leg[2] *= count_change
                count_change += 0.005
            dynamixel_positon = mapping.map([signal_leg[0],signal_leg[1],signal_leg[2],signal_leg[3],signal_leg[4]])
            cpg_walk_data = np.array(cpg_walk.update())

        if motion == "set":
            count_change = 0
            sigma = 0.03
            dynamixel_positon = mapping.map([0,0,0,1,1]) 
            cpg_walk = CPG()
            cpg_walk.set_frequency(sigma * np.pi)
        elif motion == "forward":
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 1
        elif motion == "backward":
            signal_leg[0] = -cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 1
        elif motion == "left":
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 0.5
            signal_leg[4] = 1
        elif motion == "right":
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 0.5
        elif motion == "stop":
            if count_change > 0:
                count_change -= 0.005
                if motion_before == "forward":
                    signal_leg[0] = cpg_walk_data[0] * count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 1
                elif motion_before == "backward":
                    signal_leg[0] = -cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 1
                elif motion_before == "left":
                    signal_leg[0] = cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 0.5
                    signal_leg[4] = 1
                elif motion_before == "right":
                    signal_leg[0] = cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 0.5
        if motion != "stop":
            motion_before = motion



        if speed == "+sigma":
            sigma += 0.0001
            if sigma >= 0.06: sigma = 0.06
            cpg_walk.set_frequency(sigma * np.pi)
        elif speed == "-sigma":
            sigma -= 0.0001
            if sigma <= 0.01: sigma = 0.01
            cpg_walk.set_frequency(sigma * np.pi)


        print("motion: %s"%motion)
        print("count_change: %.4f"%count_change)
        print("sigma: %.4f"%sigma)
        # learning_rate = 0.01
        # forgeting_rate = 0.01
        # if motion == "stop":
        #     state = 0
        # else
        #     state = 1
        # alpha = (learning_rate * state * (sigma + alpha)) + (forgeting_rate * (state-1) * (alpha**2))

        if motion == "set":
            arduino_control = [0,0]
            alpha = set_alpha
            shif_cpg_breathe = set_shif_cpg_breathe
            time_t0 = time.perf_counter()
            cpg_breathe = CPG()
            cpg_breathe.set_frequency()
        elif motion == "stop":
            alpha -= rate_alpha
            if alpha <= min_alpha: alpha = min_alpha
            shif_cpg_breathe -= rate_cpg_breathe
            if shif_cpg_breathe <= min_shif_cpg_breathe: shif_cpg_breathe = min_shif_cpg_breathe
            cpg_breathe.set_frequency(alpha * np.pi)
        else:
            alpha += rate_alpha
            if alpha >= max_alpha: alpha = max_alpha
            shif_cpg_breathe += rate_cpg_breathe
            if shif_cpg_breathe >= max_shif_cpg_breathe: shif_cpg_breathe = max_shif_cpg_breathe
            cpg_breathe.set_frequency(alpha * np.pi)


        if pump == True:
            arduino_control = [1,1]
            
        if motion != "set":
            
            cpg_breathe_data_0 = np.array(cpg_breathe.update())[0] + shif_cpg_breathe
            cpg_breathe_data_1 = np.array(cpg_breathe.update())[1] + shif_cpg_breathe

            if breathe_state_0 == True and cpg_breathe_data_0 >= 0:
                MOTOR1_DATA = 0
                breathe_state_0 = False
                print("----------------------------------------------------")
                print(time.perf_counter()-time_t0)
                time_t0 = time.perf_counter()
            elif breathe_state_0 == False and cpg_breathe_data_0 < 0:
                MOTOR1_DATA = 1
                breathe_state_0 = True

            if breathe_state_1 == True and cpg_breathe_data_1 >= 0:
                MOTOR2_DATA = 0
                breathe_state_1 = False
            elif breathe_state_1 == False and cpg_breathe_data_1 < 0:
                MOTOR2_DATA = 1
                breathe_state_1 = True
       
            arduino_control = [MOTOR1_DATA,MOTOR2_DATA]   
             

            print("cpg_breathe_data_0 %.4f"%cpg_breathe_data_0)
        print("alpha %.4f"%alpha)
        print("shif_cpg_breathe %.4f"%shif_cpg_breathe)       
        

        dynamixel_control_data = Int32MultiArray()
        dynamixel_control_data.data = dynamixel_positon
        pub_dynamixel.publish(dynamixel_control_data)

        arduino_control_data = Float32MultiArray()
        arduino_control_data.data = arduino_control
        pub.publish(arduino_control_data)
        rate.sleep()

        signal.signal(signal.SIGINT, keyboard_interrupt_handler)   

def joy_cb(msg):
    global motion,speed,count_motion,set_sequence,pump
    print(msg.buttons)
    #print(msg.axes)


    if msg.buttons[7] == 1:
        motion = "set"
    if msg.buttons[1] == 1:
        motion = "stop"
    elif msg.axes[1] == 1:
        motion = "forward"
    elif msg.axes[1] == -1:
        motion = "backward"
    elif msg.axes[0] == 1:
        motion ="left"
    elif msg.axes[0] == -1:
        motion ="right"

    if msg.buttons[7] == 1 or msg.buttons[1] == 1 or msg.axes[1] == 1 or msg.axes[1] == -1 or msg.axes[0] == 1 or msg.axes[0] == -1:
        set_sequence = False
        count_motion = 0

    if msg.buttons[5] == 1 :
        set_sequence = True
    
    if msg.buttons[4] == 1:
        pump = True
    else:
        pump = False


    

    if  msg.buttons[3] == 1 :
        speed = "+sigma"
    elif  msg.buttons[0] == 1 :
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
