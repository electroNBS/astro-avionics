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

log_dir = os.path.expanduser("~/flightlogs")
os.makedirs(log_dir, exist_ok=True)  # Ensure the directory exists

# constants
LOG_FILE = os.path.join(log_dir, time.strftime("arduino_%H%M%S.log"))
TIMEOUT = 20
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
DROGUE_PIN = 27
MAIN_PIN = 17

# global variables
ard_status = True
ground_alt = 0.0
current_alt = 0.0
height = 0.0
last_packet_time = time.time()

# GPIO init
GPIO.setmode(GPIO.BOARD) # Check if we are using board numbering
GPIO.setup(DROGUE_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(MAIN_PIN, GPIO.OUT, initial=GPIO.LOW)

# bmp init
i2c = busio.I2C(board.SCL, board.SDA)
bmp = adafruit_bmp390.BMP390_I2C(i2c, address=0x77) # install i2c tools and verify address with $ i2cdetect -y 1
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 2
bmp.sea_level_pressure = 1007 # Set this before flight
ground_alt = bmp.altitude

def start_recording():
    subprocess.run(["bash", "/home/pi/start_recording.sh"])

def stop_recording():
    subprocess.run(["bash", "/home/pi/stop_recording.sh"])

def deploy_drogue():
    GPIO.output(DROGUE_PIN, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(DROGUE_PIN, GPIO.LOW)
    write_log("Drogue chute deployed!")

def deploy_main():
    GPIO.output(MAIN_PIN, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(MAIN_PIN, GPIO.LOW)
    write_log("Main chute deployed!")

def write_log(data):
    global last_packet_time
    with open(LOG_FILE, "a") as file:
        file.write(f"{time.strftime('%H%M%S')} - {data}\n")
        file.flush()
    last_packet_time = time.time()

async def read_serial(aios):
    global last_packet_time
    global ard_status
    while ard_status:
        try:
            data = await aios.readline_async()
            if data:
                data = data.decode().strip()
                write_log(data)
                if data == "STOPREC":
                    stop_recording()
                elif data == "EXIT":
                    sys.exit()
        except Exception as e:
            write_log(f"Serial read error: {e}")
            await asyncio.sleep(0.5)

async def check_failure():
    global last_packet_time
    global ard_status
    while ard_status:
        if time.time() - last_packet_time > TIMEOUT:
            write_log("ERROR: Arduino timed out! Switching to failure mode.")
            stop_recording()
            ard_status = False
        await asyncio.sleep(1)  # Check every second

async def update_alt():
    global ground_alt
    global current_alt
    global height
    while True:
        current_alt = bmp.altitude
        height = current_alt - ground_alt
        write_log(f"Altitude= {current_alt}   Height= {height}")
        await asyncio.sleep(0.2)

async def failure_mode():
    global height
    for i in range(5):
        # Add beeper code here to indicate failure
        await asyncio.sleep(0.25)
    while height < 200:
        await asyncio.sleep(1)

async def flight_mode():
    global current_alt
    previous_alt = current_alt
    counter = 0
    while True:
        await asyncio.sleep(0.2)
        if current_alt < previous_alt:
            counter += 1
        if counter > 3:
            write_log("Descent detected! Deploying drogue chute.")
            deploy_drogue()
            break
        previous_alt = current_alt
        
async def descent_mode():
    global height
    while height>500:
        await asyncio.sleep(0.5)
    write_log("500m reached, deploying main chute!")
    deploy_main()


async def main():
    start_recording()
    aios = aioserial.AioSerial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    # Add gps and comms code here without awaiting
    await asyncio.gather(read_serial(aios), check_failure()) # make sure this function returns if ard_status is false
    
    # Failure mode
    write_log("Entered failure mode.")
    asyncio.create_task(update_alt())  # Add some case to stop this fn
    await failure_mode()
    await flight_mode()
    await descent_mode()

asyncio.run(main())
