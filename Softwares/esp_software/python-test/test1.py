import serial
from time import sleep

ser = serial.Serial('COM5', 115200, timeout=1, dsrdtr=True)  # Open the serial port with a baud rate of 9600

def move_to_position(x, y):
    # Function to move the servo to a specific position
    cc = "G," + str(x) + "," + str(y) + ","
    print(cc)
    ser.write(cc.encode())  # Send the command to the serial port
    sleep(0.1)

# move_to_position(0, 0)
while True:
    # for i in range(45):
    #     move_to_position(i, i)

    # sleep(1)  # Wait for 1 second before sending the next command
    
    # move_to_position(90, 90)  # Move to the initial position
    # sleep(3)  # Wait for 1 second before sending the next command

    # for i in range(90, 135):
    #     move_to_position(i, i)

    # move_to_position(180, 180)  # Move to the initial position
    # sleep(3)

    # for i in range(180, 0, -1):
    #     move_to_position(i, i)

    # move_to_position(100, 100)  # Move to the initial position
    # sleep(2)
    # for i in range(100, 0, -1):
    #     move_to_position(i, i)
        
    # sleep(2)

    move_to_position(-5, -5)
    sleep(1)
    move_to_position(-5, 0)
    sleep(1)
    move_to_position(0, 0)
    sleep(1)
    move_to_position(0, -5)
    sleep(1)
