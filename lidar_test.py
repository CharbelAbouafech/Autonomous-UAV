import serial

# Open UART port (ttyAMA3)
ser = serial.Serial(
    port='/dev/ttyAMA3',
    baudrate=115200,
    timeout=1
)

def read_tfmini():
    while True:
        # TF Mini frame starts with 0x59 0x59
        if ser.read() == b'\x59':
            if ser.read() == b'\x59':
                data = ser.read(7)  # Remaining bytes
                
                if len(data) == 7:
                    # Distance = low byte + high byte
                    dist = data[0] + (data[1] << 8)
                    
                    # Signal strength (optional)
                    strength = data[2] + (data[3] << 8)
                    
                    return dist, strength
    return None, None

try:
    while True:
        distance, strength = read_tfmini()
        
        if distance is not None:
            print(f"Distance: {distance} mm | Strength: {strength}")
            
            # Check threshold (10 mm)
            if distance <= 10:
                print("TOO CLOSE!")

except KeyboardInterrupt:
    ser.close()
    print("Stopped")