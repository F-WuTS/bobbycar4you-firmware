import serial
import struct
import time

PORT = "/dev/ttyUSB0"  
BAUD = 115200

START_FRAME = 0xABCD
TIME_SEND   = 0.1       # seconds
SPEED_MAX   = 300
SPEED_STEP  = 20

# Structs in little-endian format
# Command: uint16 start, int16 steer, int16 speed, uint16 checksum
CMD_STRUCT = "<HhhH"

# Feedback: uint16 start, int16 cmd1, int16 cmd2, int16 speedR_meas,
#           int16 speedL_meas, int16 batVoltage, int16 boardTemp,
#           uint16 cmdLed, uint16 checksum
FEEDBACK_STRUCT = "<HhhhhhhHH"

def make_command(steer, speed):
    start = START_FRAME
    # Mask to 16-bit unsigned range
    checksum = (start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
    return struct.pack(CMD_STRUCT, start, steer, speed, checksum)


def parse_feedback(packet):
    if len(packet) != struct.calcsize(FEEDBACK_STRUCT):
        return None
    fb = struct.unpack(FEEDBACK_STRUCT, packet)
    start, cmd1, cmd2, speedR, speedL, vbat, temp, led, checksum = fb
    cs = start ^ cmd1 ^ cmd2 ^ speedR ^ speedL ^ vbat ^ temp ^ led
    if start == START_FRAME and checksum == cs:
        return {
            "cmd1": cmd1,
            "cmd2": cmd2,
            "speedR": speedR,
            "speedL": speedL,
            "vbat": vbat / 10.0,  # convert to volts
            "temp": temp,
            "led": led
        }
    return None

with serial.Serial(PORT, BAUD, timeout=0.01) as ser:
    print("Hoverboard Serial v1.0 (Python)")
    iTest = 0
    step = SPEED_STEP
    last_send = time.time()

    buffer = bytearray()

    while True:
        # receive bytes
        data = ser.read(100)
        if data:
            buffer.extend(data)
            # try to find start frame 0xABCD
            while len(buffer) >= struct.calcsize(FEEDBACK_STRUCT):
                # search for 0xABCD little endian (0xCD 0xAB)
                idx = buffer.find(b'\xCD\xAB')
                if idx < 0:
                    buffer.clear()
                    break
                if idx > 0:
                    del buffer[:idx]
                if len(buffer) < struct.calcsize(FEEDBACK_STRUCT):
                    break
                packet = buffer[:struct.calcsize(FEEDBACK_STRUCT)]
                del buffer[:struct.calcsize(FEEDBACK_STRUCT)]
                fb = parse_feedback(packet)
                if fb:
                    print(f"R:{fb['speedR']} L:{fb['speedL']} "
                          f"Vbat:{fb['vbat']:.1f}V Temp:{fb['temp']}C")

        # send commands every TIME_SEND
        if time.time() - last_send >= TIME_SEND:
            pkt = make_command(0, iTest)
            ser.write(pkt)
            last_send = time.time()

            iTest += step
            if iTest >= SPEED_MAX or iTest <= -SPEED_MAX:
                step = -step