import usb.core
import usb.util

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

# Replace the endpoint addresses with your device's endpoint addresses
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

# Write data to the bulk OUT endpoint (replace data with what you want to send)
data_out = b'\x01\x02\x03\x04'  # Replace with your data
bulk_out_endpoint.write(data_out)

# Read data from the bulk IN endpoint
data_in = bulk_in_endpoint.read(512)  # Adjust the size to match your endpoint's max packet size
print("Received:", data_in)

# Cleanup
usb.util.dispose_resources(dev)