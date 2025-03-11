import subprocess

def start_recording():
    subprocess.run(["bash", "/home/pi/start_recording.sh"])

def stop_recording():
    subprocess.run(["bash", "/home/pi/stop_recording.sh"])