#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String
import time

# 전역 변수
open_detected = None

def command_start_callback(data):
    global open_detected
    if data.data == 'Start': 
        open_detected = False

def command_open_callback(data):
    global open_detected
    if data.data == 'open': 
        open_detected = True

def rotate(direction, pwm):
    if direction == 'clockwise':
        duty = 7.5 + 5  # 시계방향
    elif direction == 'counterclockwise':
        duty = 7.5 - 5  # 반시계방향
    
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # 0.5초 동안 회전
    pwm.ChangeDutyCycle(7.5)  # 초기 위치로 복귀

def main():
    global open_detected
    servo_pin = 17

    # GPIO 핀 설정
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servo_pin, GPIO.OUT)

    # PWM (50Hz)
    pwm = GPIO.PWM(servo_pin, 50)
    pwm.start(7.5)  # 초기값

    # ROS 노드 초기화
    rospy.init_node('servo_controller', anonymous=True)
    
    # 토픽 구독 설정
    rospy.Subscriber('start', String, command_start_callback)
    rospy.Subscriber('open', String, command_open_callback)
    
    try:
        # ROS 노드가 종료될 때까지 실행
        while not rospy.is_shutdown():
            if open_detected == True:
                rospy.loginfo("시계 방향으로 회전 중...")
                rotate('clockwise', pwm)
                time.sleep(0.5)
                open_detected = None
            elif open_detected == False:
                rospy.loginfo("반시계 방향으로 회전 중...")
                rotate('counterclockwise', pwm)
                time.sleep(0.5)
                open_detected = None
            else:
                # 중립값으로 유지하여 서보모터가 움직이지 않도록 함
                pwm.ChangeDutyCycle(7.5)
                time.sleep(0.1)  # 짧은 시간 대기
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main() 