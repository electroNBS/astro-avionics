import subprocess
import serial
import time

def start_recording():
    subprocess.run(["bash", "/home/pi/start_recording.sh"])

def stop_recording():
    subprocess.run(["bash", "/home/pi/stop_recording.sh"])

# Enable UART in /boot/config.txt
# Startup sequence

ard_status = False

# time.sleep(2)
ser = serial.Serial('/dev/serial0', 9600, timeout=5)

ser.write(b'PING\n')  
response = ser.readline().decode().strip()

if response == "PONG":
    ard_status = True

ser.close()

# If ard_status is true then start cameras, logging
# Else go into failure mode and only beam telemetry back