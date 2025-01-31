import usb.core
import usb.util
import struct
from joint_calculations import *

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
    def __init__(self, theta1, theta2, theta3, theta4, current1, current2, current3, W1, W2):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        

        self.current1 = current1
        self.current2 = current2
        self.current3 = current3

        self.W1 = W1
        self.W2 = W2

    @classmethod
    def deserialize(cls, byte_array):
        # Unpack Data
        theta1, theta2, theta3, theta4, current1, current2, current3, W1, W2 = struct.unpack('fffffffff', byte_array)
        # Return an instance of ArmStatus with the unpacked values

    

        return cls(theta1, theta2, theta3, theta4, current1, current2, current3,W1,W2)
    
    def __str__(self):
    # Define a nicely formatted string for printing
        return (f"ArmStatus:\n"
            f"  Theta 1: {self.theta1:.2f}\n"
            f"  Theta 2: {self.theta2:.2f}\n"
            f"  Theta 3: {self.theta3:.2f}\n"
            f"  Theta 4 {self.theta4:.2f}\n"
            f"  W1: {self.W1:.2f}\n"
            f"  W2: {self.W2:.2f}")





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

    data_in = bulk_in_endpoint.read(512)  # Adjust the size to match your endpoint's max packet size
    arm_status = ArmStatus.deserialize(data_in)
    #Data out
    TorqueCommand.T1, TorqueCommand.T2, TorqueCommand.T3, x,y,z = deg2torque(arm_status.theta1, arm_status.theta2, arm_status.theta3,arm_status.W1,arm_status.W2)
    
    # Read data from the bulk IN endpoint
    bulk_out_endpoint.write(TorqueCommand.serialize())




    # print(arm_status)

    print(TorqueCommand.T1, TorqueCommand.T2, TorqueCommand.T3, x,y,z)
    # print(arm_status.theta1, arm_status.theta2, arm_status.theta3, arm_status.theta4)
    
    # Write data to the bulk OUT endpoint (replace data with what you want to send)

# Cleanup
usb.util.dispose_resources(dev)