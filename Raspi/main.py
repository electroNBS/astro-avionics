import asyncio
import aioserial
import subprocess
import time
import sys
import os
import board
import busio
import adafruit_bmp390
import RPi.GPIO as GPIO
# pip install adafruit-circuitpython-bmp390
# enable i2c and serial in raspi config

# define log directory
log_dir = os.path.expanduser("~/flightlogs")
os.makedirs(log_dir, exist_ok=True)  # Ensure the directory exists

# bmp init
i2c = busio.I2C(board.SCL, board.SDA)
bmp = adafruit_bmp390.BMP390_I2C(i2c, address=0x77) # install i2c tools and verify address with $ i2cdetect -y 1
#bmp.pressure_oversampling = 8
#bmp.temperature_oversampling = 4
bmp.sea_level_pressure = 1007 # Set this before flight!

# constants
LOG_FILE = os.path.join(log_dir, time.strftime("arduino_%H%M%S.log"))
TIMEOUT = 20
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
DROGUE_PIN = 27
MAIN_PIN = 17
BACKUP_PIN = 22
STATUS = 25
TEL = 24
MAIN_VEL = -40 # This should be negative!
MAIN_HEIGHT = 500
FLIGHTMODE_ALT = 200
GROUND_ALT = bmp.altitude

# GPIO init
GPIO.setmode(GPIO.BCM) 
GPIO.setup(DROGUE_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(MAIN_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(STATUS, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(TEL, GPIO.OUT, initial=GPIO.LOW)

# global variables
ard_status = True
vel_logging = False
current_alt = 0.0
height = 0.0
velocity = 0.0
last_packet_time = time.monotonic()

def start_recording():
    script_path = "/home/pi/start_recording.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
         write_to_file("ERROR: start_recording.sh not found!")

def stop_recording():
    script_path = "/home/pi/stop_recording.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
        write_to_file("ERROR: stop_recording.sh not found!")

async def deploy_drogue():
    GPIO.output(DROGUE_PIN, GPIO.HIGH)
    await asyncio.sleep(2)
    GPIO.output(DROGUE_PIN, GPIO.LOW)
    await write_log("Drogue chute deployed!")

async def deploy_main():
    GPIO.output(MAIN_PIN, GPIO.HIGH)
    await asyncio.sleep(2)
    GPIO.output(MAIN_PIN, GPIO.LOW)
    await write_log("Main chute deployed!")

async def write_log(data):
    global last_packet_time
    await asyncio.to_thread(write_to_file, data)
    last_packet_time = time.monotonic()

def write_to_file(data):
    with open(LOG_FILE, "a") as file:
        file.write(f"{time.strftime('%H:%M:%S')} - {data}\n")
        file.flush()

async def read_serial(aios):
    while ard_status:
        try:
            data = await aios.readline_async()
            if not data:
                continue

            data = data.decode().strip()
            await write_log(data)
            if data == "STOPREC":
                stop_recording()
            elif data == "EXIT":
                await cleanup(aios)
                sys.exit()

        except Exception as e:
            await write_log(f"Serial read error: {e}")
            await asyncio.sleep(0.5)

async def check_failure():
    global last_packet_time, ard_status
    while True:
        if time.monotonic() - last_packet_time > TIMEOUT:
            await write_log("ERROR: Arduino timed out! Switching to failure mode.")
            stop_recording()
            ard_status = False
            return
        await asyncio.sleep(1)  # Check every second

async def update_vel():
    global current_alt, height, velocity, vel_logging
    await write_log(f"Started velocity loggging. Ground altitude: {GROUND_ALT}")
    previous_alt = bmp.altitude
    previous_time = time.monotonic()
    delta_alt = 0.0
    delta_time = 1.0
    counter = 0
    vel_logging = True
    while vel_logging:
        current_alt = bmp.altitude
        current_time = time.monotonic()
        height = current_alt - GROUND_ALT

        delta_alt = current_alt - previous_alt
        delta_time = current_time - previous_time
        if delta_time > 0:
            velocity = delta_alt / delta_time
        previous_alt = current_alt
        previous_time = current_time

        # Write to log every second
        counter += 1
        if counter > 50:
            await write_log(f"Altitude= {current_alt:.2f}  Height= {height:.2f}  Velocity= {velocity:.2f}")
            counter = 0
        await asyncio.sleep(0.005)
    await write_log("Stopping velocity logging.")

async def failure_mode():
    while height < FLIGHTMODE_ALT:
        await asyncio.sleep(0.25)

async def flight_mode():
    counter = 0
    while True:
        if velocity < 0:
            counter += 1
        else:
            counter = 0
        if counter > 6:
            break
        await asyncio.sleep(0.005)
    await write_log("Apogee detected! Deploying drogue.")
    await deploy_drogue()
        
async def descent_mode():
    while height > MAIN_HEIGHT and velocity < MAIN_VEL:
        await asyncio.sleep(0.1)
    await write_log(f"{MAIN_HEIGHT}m reached, deploying main chute!")
    await deploy_main()

async def landing_mode():
    counter = 0
    while counter < 10:
        if abs(velocity) < 0.5:
            counter += 1
        else:
            counter = 0
        await asyncio.sleep(0.5)
    write_log("Landing detected!")

async def cleanup(aios):
    global vel_logging
    vel_logging = False
    GPIO.output(STATUS, GPIO.LOW)
    GPIO.output(TEL, GPIO.LOW)
    GPIO.cleanup()
    await aios.close()
    await write_log("Cleaned up serial, GPIO and stopped velocity logging. Exiting!")

async def main():
    GPIO.output(STATUS, GPIO.HIGH)
    GPIO.output(TEL, GPIO.HIGH)
    start_recording()
    aios = aioserial.AioSerial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    await asyncio.gather(read_serial(aios), check_failure()) # make sure this function returns if ard_status is false
    
    # Failure mode
    await write_log("Entered failure mode.")
    GPIO.output(TEL, GPIO.LOW)
    vel_task = asyncio.create_task(update_vel())
    await failure_mode()
    await flight_mode()
    await descent_mode()
    await landing_mode()
    await cleanup(aios)    
    await vel_task

if __name__ == "__main__":
    asyncio.run(main())
