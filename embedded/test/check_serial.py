import serial

# Open the serial port with a timeout of 5 seconds
ser = serial.Serial('/dev/ttyACM0', timeout=1)

try:
   while True:
       # Read a line from the serial port
       line = ser.readline()
       print(line)
except serial.serialutil.SerialException as e:
   print("An error occurred: ", str(e))