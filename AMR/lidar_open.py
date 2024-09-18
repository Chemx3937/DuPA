import os
import time
import subprocess
import RPi.GPIO as GPIO

class LidarControl:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        self.process = None

    def radar_open(self):
        def radar_high():
            time.sleep(0.1)
            GPIO.output(20, GPIO.HIGH)

        radar_high()
        time.sleep(0.05)
        # Launch only if not already running
        if not self.is_running("agv_start.launch"):
            launch_command = "roslaunch myagv_odometry agv_start.launch"
            self.process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f"{launch_command}; exec $SHELL"])

    def radar_close(self):
        def radar_low():
            time.sleep(0.1)
            GPIO.output(20, GPIO.LOW)

        radar_low()
        time.sleep(0.05)
        if self.process:
            self.process.terminate()
            self.process.wait()
        close_command = "ps -ef | grep -E 'agv_start.launch' | grep -v 'grep' | awk '{print $2}' | xargs kill -2"
        subprocess.run(close_command, shell=True)

    def is_running(self, launch_file):
        result = subprocess.run(["pgrep", "-f", launch_file], stdout=subprocess.PIPE)
        return len(result.stdout) > 0

    def close(self):
        GPIO.cleanup()

if __name__ == "__main__":
    lidar_control = LidarControl()
    
    try:
        lidar_control.radar_open()
        print("Radar is opened. Checking ROS topics...")
        time.sleep(10)
        os.system("rostopic list")
        
        input("Press Enter to close the radar...")

        lidar_control.radar_close()
        print("Radar is closed. Checking ROS topics...")
        time.sleep(5)
        os.system("rostopic list")
    
    finally:
        lidar_control.close()