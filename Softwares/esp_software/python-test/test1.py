import serial, random
from time import sleep

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, dsrdtr=True)  # Open the serial port with a baud rate of 9600

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

    # move_arr = [(-40, -12), (-40, -2), (-30, -2), (-30, -12), (-30, -17), (-40, -17)]

    move_arr = [(5,7), (5,2), (5, -8), (17, -8), (17, 2), (17, 7)]

    # old_pos = (0, 0)
    # for i in range(len(move_arr)):
    #     pos = random.randint(0, len(move_arr)-1)
    #     if pos != old_pos:
    #         old_pos = pos
    #         move_to_position(move_arr[pos][0], move_arr[pos][1])
    #         sleep(2)

    #         while ser.in_waiting:
    #             data = ser.readline().decode().strip()
    #             print(f"Received: {data}")

    for pos in move_arr:
        move_to_position(pos[0], pos[1])
        sleep(2)
        # print received data
        while ser.in_waiting:
            data = ser.readline().decode().strip()
            print(f"Received: {data}")
