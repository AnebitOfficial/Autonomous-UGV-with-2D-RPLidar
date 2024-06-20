'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
import pyfirmata
import time 


def move_servo(angle):
    pin9.write(angle)


PORT_NAME = '/dev/ttyUSB0'
board = pyfirmata.Arduino('/dev/ttyACM0')

iter8 = pyfirmata.util.Iterator(board)
iter8.start()
pin9 = board.get_pin('d:12:s')


print("starting")

while True:
    angle = eval(input("Enter angle in degrees (90 is parallel to floor):"))
    move_servo(angle)





