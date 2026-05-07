from smbus2 import SMBus
import time

# Get I2C bus - Using SMBus(1) directly
bus = SMBus(1)

# ISL29125 address: 0x44
# Register 0x01: Configure RGB Mode
# Register 0x09: Sync/Integration time
bus.write_byte_data(0x44, 0x01, 0x05) # Enable RGB mode
time.sleep(0.5)

print("Reading colour values... Press Ctrl+C to stop\n")

def getAndUpdateColour():
    try:
        while True:
            # Read 6 bytes starting from Register 0x09
            # Order: Green LSB, Green MSB, Red LSB, Red MSB, Blue LSB, Blue MSB
            data = bus.read_i2c_block_data(0x44, 0x09, 6)

            # Convert bytes to 16-bit integers
            green = (data[1] << 8) | data[0]
            red   = (data[3] << 8) | data[2]
            blue  = (data[5] << 8) | data[4]

            # calibration adjustment (if needed)
            red = int(red * 1.3)/1000
            green = int(green * 0.75)/1000
            blue = int(blue * 1.25)/1000

            print(f"RGB Values -> Red: {red} | Green: {green} | Blue: {blue}")
            
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
        bus.close()

# Start the loop
getAndUpdateColour()