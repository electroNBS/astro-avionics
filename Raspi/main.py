import subprocess
import serial
import time
import sys

def start_recording():
    subprocess.run(["bash", "/home/pi/start_recording.sh"])

def stop_recording():
    subprocess.run(["bash", "/home/pi/stop_recording.sh"])

def write_log(data):
    global last_packet_time
    with open(LOG_FILE, "a") as file:
        file.write(f"{time.strftime('%H%M%S')} - {data}\n")
        file.flush()

    last_packet_time = time.time()
    ser.write(b'ACK\n')  # Send acknowledgment to Arduino

def check_failure():
    if time.time() - last_packet_time > TIMEOUT:
        print("ERROR: Arduino timed out! Switching to failure mode.")
        return False
    return True

LOG_FILE = time.strftime("arduino_%H%M%S.log")
TIMEOUT = 20

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
ard_status = True
last_packet_time = time.time()
data = ""

# Normal mode
start_recording()

while ard_status:
    if ser.in_waiting:
        data = ser.readline().decode().strip()
        if data:
            write_log(data)

    ard_status = check_failure()

    if data == "STOPREC" or not ard_status:
        stop_recording()
    
    if data == "EXIT":
        sys.exit()

    # Add drogue chute and main chute deployment code below

    time.sleep(1)

# Failure mode, arduino took longer than TIMEOUT