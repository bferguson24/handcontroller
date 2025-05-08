import usb.core
import usb.util
import struct
import time
# from joint_calculations import *
import json
# import helpers


# helpers.do_help()


from websockets.sync.client import connect
websocket = connect("ws://localhost:8081")
print('Connected')

# Define the Python equivalent of the C struct
class SetpointCmd:
    def __init__(self):
        self.T1 = 0
        self.T2 = 0
        self.T3 = 0

    def serialize(self):
        # Pack the three floats (j1_torque, j2_torque, j3_torque) into a byte array
        return struct.pack('fff', self.T1, self.T2, self.T3)
    
    
class ArmStatus:
    def __init__(self, x, y , z, elbow_angle, clutch_status, trigger_amount):

    #Floats
        self.x = x
        self.y = y
        self.z = z
        self.elbow_angle = elbow_angle
        self.trigger_amount = trigger_amount

    #Bool
        self.clutch_status = clutch_status  

    @classmethod
    def deserialize(cls, byte_array):
        # Unpack Data
        x, y, z, elbow_angle, trigger_amount, clutch_status = struct.unpack('fffff?', byte_array)
        # Return an instance of ArmStatus with the unpacked values


        return cls(x, y, z, elbow_angle, trigger_amount, clutch_status)
    
    def __str__(self):
        return (f"Arm Status:\n"
                f"  X: {self.x:.2f}\n"
                f"  Y: {self.y:.2f}\n"
                f"  Z: {self.z:.2f}\n"
                f"  Elbow Angle: {self.elbow_angle:.2f}\n"
                f"  Trigger Amount: {self.trigger_amount:.2f}\n"
                f"  Clutch Status: {'CLUTCH ON' if self.clutch_status else 'CLUTCH OFF'}")


def read_write_data(size_data_in, torque_out): 
    data_in = bulk_in_endpoint.read(size_data_in)  # Adjust the size to match your endpoint's max packet size
    arm_status = ArmStatus.deserialize(data_in)

    bulk_out_endpoint.write(torque_out.serialize())
    return arm_status

def average_data(size_data_in, data_out, sample_size):
    total_x = total_y = total_z = total_elbow = total_trigger = 0.0
    
    for _ in range(sample_size):

        arm_status = read_write_data(size_data_in, data_out) 

        total_x += arm_status.x
        total_y += arm_status.y
        total_z += arm_status.z 
        total_elbow += arm_status.elbow_angle


        avg_x = total_x / sample_size
        avg_y = total_y / sample_size
        avg_z = total_z / sample_size
        avg_ea = total_elbow / sample_size

        clutch = arm_status.clutch_status
        trigger = arm_status.trigger_amount


    return ArmStatus(avg_x, avg_y, avg_z, avg_ea, clutch, trigger)
        


# Example usage

# Find the USB device (replace with your device's VID and PID)
VENDOR_ID = 0xCafe  # Replace with your device's Vendor ID
PRODUCT_ID = 0x4000  # Replace with your device's Product ID

# Find the device
dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

if dev is None:
    raise ValueError("Device not found")

# Set the active configuration (if needed)
dev.set_configuration()

# Get the first configuration
cfg = dev.get_active_configuration()

# Find the bulk IN and OUT endpoints
interface = cfg[(0, 0)]  # (interface, alternate_setting)

TorqueCommand = SetpointCmd()



# Replace the endpoint addresses with your device's endpoint addresses
TorqueCommand = SetpointCmd()

bulk_out_endpoint = usb.util.find_descriptor( 
    interface,
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
)

bulk_in_endpoint = usb.util.find_descriptor(
    interface,
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
)

if bulk_out_endpoint is None or bulk_in_endpoint is None:
    raise ValueError("Bulk endpoints not found")
bulk_out_endpoint.write(TorqueCommand.serialize())

while(1):

  
    arm_status = read_write_data(21, TorqueCommand)

    # data_in = bulk_in_endpoint.read(21)  # Adjust the size to match your endpoint's max packet size
    # arm_status = ArmStatus.deserialize(data_in)

    avg_arm_status = average_data(21,TorqueCommand, 20)
    print(round(avg_arm_status.x), round(avg_arm_status.y), round(avg_arm_status.z), round(avg_arm_status.elbow_angle), round(avg_arm_status.trigger_amount), round(avg_arm_status.clutch_status))

    # bulk_out_endpoint.write(TorqueCommand.serialize())

    message = {
        "jsonrpc": "2.0",
        "method": "set_mira_cartesian",
        "params": 
            {
                "left": [70.0, 28.0, -76.7, 0.0],
                "right": [avg_arm_status.x, avg_arm_status.y, avg_arm_status.z, avg_arm_status.elbow_angle]
            },
        "id": 0
    }

    websocket.send(json.dumps(message))

    time.sleep(0.01)
    

# Cleanup
usb.util.dispose_resources(dev)