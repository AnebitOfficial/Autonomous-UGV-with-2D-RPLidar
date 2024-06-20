from pynput.keyboard import Key, Listener
from pyfirmata import Arduino, util
import time
import math
import subprocess
import rospy
from geometry_msgs.msg import PoseStamped
import threading
import signal
import sys
import os
import numpy as np

global LS,RS,it,x_pose,y_pose,z_pose,w_pose
LS, RS = 0,0
it = 0.05

spin_thread = None
roscore = 'roscore'
bash = 'echo 1604 | sudo -S chmod 666 /dev/ttyUSB0; exec bash'
lidar = 'roslaunch rplidar_ros rplidar_a1.launch'
hector = 'roslaunch hector_slam_launch tutorial.launch'
readpos = 'source ~/catkin_ws/devel/setup.bash; exec rostopic echo /slam_out_pose'

# Adjust this to match your Arduino's port
board = Arduino('/dev/ttyACM0')  # Adjusted to use COM9

# Define motor pins
left_motor_forward = board.get_pin('d:9:o')  # IN1
left_motor_backward = board.get_pin('d:10:o')  # IN2
right_motor_forward = board.get_pin('d:6:o')  # IN3
right_motor_backward = board.get_pin('d:5:o')

left_motor_EA = board.get_pin('d:3:p')  # PWM pin for motor A speed
right_motor_EA = board.get_pin('d:11:p')  # PWM pin for motor B speed

def run_in_new_terminal(command):
    """
    Opens a new terminal window and runs the specified command.
    """
    s = 'nohup ' + command  +  '> nohup.txt &'
    process = subprocess.Popen(s, shell=True,stdin=subprocess.PIPE)
    time.sleep(5)

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

   

def on_press(key):
    global LS,RS,it
    try:
        if key.char == 'w':
            LS = min(1,LS+it)
            RS = min(1,RS+it)
        elif key.char == 'a':
            LS = max(-1,LS-it/2)
            RS = min(1,RS+it/2)
        elif key.char == 's':
            LS = max(-1,LS-it)
            RS = max(-1,RS-it)
        elif key.char == 'd':
            LS = min(1,LS+it/2)
            RS = max(-1,RS-it/2)
        elif key.char == 'x':
            LS,RS = 0,0
        
        print("LS,RS:",LS*0.9,",",RS)
        move_robot(LS*0.9,RS)
    except AttributeError:
        pass  # Handle special key presses here if necessary

def on_release(key):
    global LS,RS
    if key == Key.esc:
        # Stop listener
        move_robot(0,0)
        print("Exiting")
        return False

def read_current_position(count):#read the current position using slam
    global x_pose,y_pose,z_pose,w_pose
    x,y,teta = 0,0,0
    for i in range(count):
        x += x_pose
        y += y_pose
        if z_pose > 0:
            sign = 1
        else:
            sign = -1
        teta += math.acos(w_pose)*2*sign
    x /= count
    y /= count
    teta /= count
    teta = teta/math.pi*180

    print("(x,y,teta) = (", x , "," , y , "," , teta , ") \n")
        
    return x,y,teta


def pose_callback(data):
    global x_pose,y_pose,z_pose,w_pose
    x_pose = data.pose.position.x
    y_pose = data.pose.position.y
    z_pose = data.pose.orientation.z
    w_pose = data.pose.orientation.w

def spin_fnc():
    rospy.spin()

def start_slam():
    global roscore, lidar, hector,spin_thread
    run_in_new_terminal(roscore)
    run_in_new_terminal(lidar)
    run_in_new_terminal(hector)
    time.sleep(10)

    rospy.init_node('lidar_pose_subscriber', anonymous=True) # Initialize ROS node
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback) # Subscribe to pose topic
    spin_thread = threading.Thread(target=spin_fnc)
    spin_thread.start()
    print("Slam is started")

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



def stop_slam():
    global roscore, lidar, hector,spin_thread                                         
    kill_terminal("roscore")
    kill_terminal("rplidar_a1.launch")
    kill_terminal('hector_slam_launch tutorial.launch')
    rospy.signal_shutdown('Reason for shutdown')
    if spin_thread is not None:
        spin_thread.join()
        spin_thread = None

    print("Slam is stopped")


slam = False
save = False
if sys.argv[1].upper() == 'Y':
  print("Opening Slam")
  start_slam()
  slam = True
elif sys.argv[1].upper() == "YS":
  print("Opening Slam")
  start_slam()
  slam = True
  save = True

# Collect events until released
with Listener(on_press=on_press, on_release=on_release) as listener:
    print("Started")
    listener.join()
if slam:
    x,y,teta = read_current_position(20)
    if save:
        run_in_new_terminal("rosrun map_server map_saver -f /home/ubuntu/anebit/maps/hector_map")
        time.sleep(5)
        run_in_new_terminal("python3 maps/hector_map_to_txt.py")
        run_in_new_terminal("cp /home/ubuntu/anebit/maps/hector_map.pgm ~/catkin_ws/src/laser_scan_matcher/maps/")
        run_in_new_terminal("cp /home/ubuntu/anebit/maps/hector_map.yaml ~/catkin_ws/src/laser_scan_matcher/maps/")
    stop_slam()
    time.sleep(2)
    base = open("maps/base.txt","r")
    s = base.read()
    base.close()
    lst = s.split(',')
    x += float(lst[0])
    y += float(lst[1])
    teta += float(lst[2])
    s = str(x) + ',' + str(y) + ',' + str(teta)
    print("Base location:", s)
    base = open("maps/base.txt","w")
    base.write(s)
    base.close() 
    

