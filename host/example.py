import serial
import sys
import os
from pewpew import MessageType, ProtocolParser, execute_segments


if len(sys.argv) < 2:
    print("Usage: example.py <path to usb device>")
    exit()

# Connect to the USB serial port given as the last argument, and perform a device
# handshake.
ser = serial.Serial(sys.argv[-1])
parser = ProtocolParser.connect_to_port(ser)

# Build a list of segments that moves some distance along axis 0 and then returns to the origin.
# This has no acceleration planning or idea of actual steps per mm, and thus may be dangerous to
# run on real hardware.
speed = 0.001 # 0.001 steps per microsecond
steps = 1000  
events = [(MessageType.SEGMENT, (0,0, 0.001, 0.001, steps, 0)),(MessageType.SEGMENT, (1,0, 0.001, 0.001, 0, 0))]

# Execute the list of events, yielding a generator of status updates. This must be consumed in
# real time, or the host code will stall.
for response in execute_segments(parser, iter(events)):
    print(response)

# Close the serial port, and put the device into a reset mode.
ser.close()




