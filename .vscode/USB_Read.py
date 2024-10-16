import json
import helpers
import serial
# helpers.do_help()

from websockets.sync.client import connect
websocket = connect("ws://localhost:8888")
print('Connected')

ser = serial.Serial('COM1',11520)

def read_from_usb():
    """Reads data from the USB CDC serial port."""
    try:
        if ser.in_waiting > 0:
            # Read a line of data sent by the STM32
            data = ser.readline().decode('utf-8').strip()
            return data
    except Exception as e:
        print(f"Error reading from USB: {e}")
    return None

def process_data(usb_data):
    



# Main loop to continuously read data from USB
while True:
    usb_data = read_from_usb()
    if usb_data:
    #Read And Send Data
    x = 0 
    y = 0
    z = 0
    theta4 = 0
    websocket.send(json.dumps({'data': [x,y,z,theta4]}))


