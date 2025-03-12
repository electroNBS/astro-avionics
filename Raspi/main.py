import asyncio
import aioserial
import subprocess
import time
import sys

LOG_FILE = time.strftime("arduino_%H%M%S.log")
TIMEOUT = 20
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200
ard_status = True

last_packet_time = time.time()

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

async def read_serial(aios):
    global last_packet_time
    global ard_status
    while ard_status:
        try:
            data = await aios.readline_async()
            if data:
                data = data.decode().strip()
                write_log(data)
                # await aios.write_async(b'ACK\n')  # Send ACK to Arduino
                
                if data == "STOPREC":
                    stop_recording()
                elif data == "EXIT":
                    sys.exit()
        except Exception as e:
            print(f"Serial read error: {e}")
            await asyncio.sleep(0.5)

async def check_failure():
    global last_packet_time
    global ard_status
    while ard_status:
        if time.time() - last_packet_time > TIMEOUT:
            print("ERROR: Arduino timed out! Switching to failure mode.")
            stop_recording()
            ard_status = False
        await asyncio.sleep(1)  # Check every second


async def main():
    start_recording()
    aios = aioserial.AioSerial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    await asyncio.gather(read_serial(aios), check_failure()) # make sure this function returns if ard_status is false
    # write failure code here

asyncio.run(main())
