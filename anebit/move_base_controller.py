import math
from pyfirmata import Arduino, util
import time
import subprocess
import rospy
from geometry_msgs.msg import Twist
import threading
import signal
import sys
import os
import numpy as np

# Setup

spin_thread = None
roscore = 'source ~/catkin_ws/devel/setup.bash; exec roscore'
bash = 'echo 1604 | sudo -S chmod 666 /dev/ttyUSB1; exec bash'
amcl = 'source ~/catkin_ws/devel/setup.bash; exec roslaunch laser_scan_matcher locnew2.launch'
move_base = 'source ~/catkin_ws/devel/setup.bash; exec roslaunch husky_navigation move_base.launch'


board = Arduino('/dev/ttyACM0')  # Adjust this to your Arduino's serial port

it = util.Iterator(board)

it.start()

global leftMotorSpeed, rightMotorSpeed
global lin,ang, W


# Motor pins (adjust these to match your setup)

left_motor_forward = board.get_pin('d:9:o')  # pin for left motor forward

left_motor_backward = board.get_pin('d:10:o')  # pin for left motor backward

right_motor_forward = board.get_pin('d:6:o')  # pin for right motor forward

right_motor_backward = board.get_pin('d:5:o')  # pin for right motor backward

left_motor_EA = board.get_pin('d:3:p') # PWM pin for left motor speed (ENB)
right_motor_EA = board.get_pin('d:11:p')  # PWM pin for right motor speed (ENB)



def run_in_new_terminal(command,wait_or_not=True):
    s = 'nohup ' + command  +  '> nohup.txt &'
    process = subprocess.Popen(s, shell=True,stdin=subprocess.PIPE)
    if wait_or_not:
        process.wait()


# Function to control motors

def move_robot(left_speed, right_speed):

    if left_speed < 0: #left motor should run forward, check the motor directions
        left_motor_forward.write(0)
        left_motor_backward.write(1)
    else:
        left_motor_forward.write(1)
        left_motor_backward.write(0)

    if  right_speed < 0: #right motor should run forward, check the motor directions
        right_motor_forward.write(0)
        right_motor_backward.write(1)
    else:
        right_motor_forward.write(1)
        right_motor_backward.write(0)     

    left_motor_EA.write(abs(left_speed))# PyFirmata uses values between 0 and 1 for analogWrite
    right_motor_EA.write(abs(right_speed))


    # Stop movement



def pose_callback(cmd_msg):
    global leftMotorSpeed,rightMotorSpeed, lin,ang, W
    linear = cmd_msg.linear.x
    angular = cmd_msg.angular.z
    
    if (linear == 0) and (angular != 0):
        angular = 2/W * 0.18
    
    #Example conversion for differential drive
    leftMotorSpeed = linear - angular*W/2
    rightMotorSpeed = linear + angular*W/2

    lin = cmd_msg.linear.x
    ang = cmd_msg.angular.z
    


def spin_fnc():
    rospy.spin()

def start_navigation():
    global roscore, amcl, move_base,spin_thread
    run_in_new_terminal(roscore,False)
    time.sleep(5)
    run_in_new_terminal(amcl,False)
    time.sleep(5)
    run_in_new_terminal(move_base,False)
    time.sleep(10)

    rospy.init_node('speed_subscriber', anonymous=True) # Initialize ROS node
    rospy.Subscriber("/cmd_vel", Twist, pose_callback) # Subscribe to pose topic
    spin_thread = threading.Thread(target=spin_fnc)
    spin_thread.start()
    print("Navigation is started")

def kill_terminal(command):
    try:
        c = 'ps aux | grep ' + "\'" + command + "\'"
        result = subprocess.run(c, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        output = result.stdout
        if len(output.split()) < 1:
            return
        pid = output.split()[1]
        time.sleep(2)
        subprocess.run(['kill',pid])
        return pid
    except subprocess.CalledProcessError:
        print(command,': No terminal processes found')
        return None



def stop_navigation():
    global spin_thread
    move_robot(0,0)
    kill_terminal("roscore")
    kill_terminal("locnew2.launch")
    kill_terminal('move_base.launch')
    rospy.signal_shutdown('Reason for shutdown')
    if spin_thread is not None:
        spin_thread.join()
        spin_thread = None

    print("Navigation is stopped")

def signal_handler(sig, frame):
    global run_program
    print('SIGINT (Control+C) captured, shutting down...')
    run_program = False

def main():
    global leftMotorSpeed,rightMotorSpeed, run_program, lin, ang, W
    lin = 0
    ang = 0
    W = 0.24
    
    leftMotorSpeed,rightMotorSpeed = 0,0
    run_program = True
    signal.signal(signal.SIGINT, signal_handler)
    move_robot(0,0)
    i = 0
    try:
        start_navigation()
        while run_program:
            move_robot(leftMotorSpeed,rightMotorSpeed)
            '''
            i+=1
            if i % 1000 == 0: 
                print("LS,RS:",leftMotorSpeed,",",rightMotorSpeed)
                print("lin,ang:",lin,",",ang)
            '''
        stop_navigation()
    except KeyboardInterrupt:
        stop_navigation()



if __name__ == "__main__":
    main()



