from pyfirmata import Arduino, util
import time

# Adjust this to match your Arduino's port
board = Arduino('/dev/ttyACM0')  # Adjusted to use COM9

# Define motor pins
leftMotorForward = board.get_pin('d:9:o')  # IN1
leftMotorBackward = board.get_pin('d:10:o')  # IN2
rightMotorForward = board.get_pin('d:6:o')  # IN3
rightMotorBackward = board.get_pin('d:5:o')

leftSpeed = board.get_pin('d:3:p')  # PWM pin for motor A speed
rightSpeed = board.get_pin('d:11:p')  # PWM pin for motor B speed


def stop():
    leftMotorForward.write(0)
    leftMotorBackward.write(0)
    rightMotorForward.write(0)
    rightMotorBackward.write(0)

highSpeed = 1
lowSpeed = 0.90

try:
    while True:
        command = input("Enter command (direction / duration / speed): ")
        lst = command.split(" ")
        command = lst[0].upper()
        if len(lst) >= 2:
            t = float(lst[1])
        else:
            t = 1
        if len(lst) == 3:
            lowSpeed = float(lst[2])
        if command == "LFF":
            leftMotorForward.write(1)
            leftSpeed.write(highSpeed)
        elif command == "LFS":
            leftMotorForward.write(1)
            leftSpeed.write(lowSpeed)
        elif command == "LBF":
            leftMotorBackward.write(1)
            leftSpeed.write(highSpeed)
        elif command == "LBS":
            leftMotorBackward.write(1)
            leftSpeed.write(lowSpeed)
        elif command == "RFF":
            rightMotorForward.write(1)
            rightSpeed.write(highSpeed)
        elif command == "RFS":
            rightMotorForward.write(1)
            rightSpeed.write(lowSpeed)
        elif command == "RBF":
            rightMotorBackward.write(1)
            rightSpeed.write(highSpeed)
        elif command == "RBS":
            rightMotorBackward.write(1)
            rightSpeed.write(lowSpeed)
        elif command == "FF":
            leftMotorForward.write(1)
            leftSpeed.write(highSpeed)
            rightMotorForward.write(1)
            rightSpeed.write(highSpeed)
        elif command == "FS":
            leftMotorForward.write(1)
            leftSpeed.write(lowSpeed)
            rightMotorForward.write(1)
            rightSpeed.write(lowSpeed)
        elif command == "BF":
            leftMotorBackward.write(1)
            leftSpeed.write(highSpeed)
            rightMotorBackward.write(1)
            rightSpeed.write(highSpeed)
        elif command == "BS":
            leftMotorBackward.write(1)
            leftSpeed.write(lowSpeed)
            rightMotorBackward.write(1)
            rightSpeed.write(lowSpeed)
        elif command == "RCW":
            leftMotorBackward.write(1)
            leftSpeed.write(lowSpeed)
            rightMotorForward.write(1)
            rightSpeed.write(lowSpeed)
        elif command == "RCCW":
            leftMotorForward.write(1)
            leftSpeed.write(lowSpeed)
            rightMotorBackward.write(1)
            rightSpeed.write(lowSpeed)
        else:
            stop()
        time.sleep(t)
        stop()

except KeyboardInterrupt:
    stop()
    print("Program exited.")
finally:
    board.exit()
