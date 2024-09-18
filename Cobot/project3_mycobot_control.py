#!/usr/bin/env python3
import rospy  # ROS 파이썬 인터페이스
import math  # 수학 라이브러리
import time  # 시간 라이브러리
import queue  # 큐 라이브러리
import numpy as np  # NumPy 라이브러리
from geometry_msgs.msg import Vector3  # ROS 벡터3 메시지
from std_msgs.msg import String, Int32  # ROS 문자열 및 정수 메시지
from pymycobot.mycobot import MyCobot  # MyCobot 라이브러리

##################        초기값        ####################
speed = 10  # 속도 설정
model = 0  # 모델 설정
med = []  # 약 목록 초기화
cleaned_med = []  # 정리된 약 목록 초기화
tvec = []  # 변위 벡터 초기화
finish_id = []  # 완료된 ID 목록 초기화
marker_id = 0  # 마커 ID 초기화
aruco_X = 0  # ArUco X 좌표 초기화
aruco_Y = 0  # ArUco Y 좌표 초기화
tracking = True  # 추적 상태 초기화
detecting = True  # 검출 상태 초기화

angles_A        = [-41,  59,  25,   4, -90, -32]  # 약 A의 각도 설정
angles_A_mid    = [-41,  43,  25,  12, -90, -32]  # 약 A 중간 각도 설정

angles_B        = [-52,  21,  91, -21, -90, -45]  # 약 B의 각도 설정
angles_B_mid    = [-50,   0,  91, -12, -90, -45]  # 약 B 중간 각도 설정

angles_C        = [-70,   0, 100, -10, -92, -60]  # 약 C의 각도 설정
angles_C_mid    = [-70, -20, 100, -10, -92, -60]  # 약 C 중간 각도 설정

angles_pha      = [ 50,  63,  18,   4, -90,   0]  # 약사 위치 각도 설정
angles_pha_mid  = [ 50,  40,  40,   0, -90,   0]  # 약사 위치 중간 각도 설정

angles_mid_area = [-6,   11,  30,  46, -90,   0]  # 중간 위치 각도 설정

#################         Tracking         #################
#약사가 약통을 반납하면, 그 약통을 트래킹하여 보관함으로 돌려놓는 동작
def tracking_ArUco(i, k): # i는 아루코마커의 x값, k는 아루코마커의 y값에 해당
    global tracking
    if tracking == True:
        alpha_degree = 15
        alpha_radians = math.radians(alpha_degree)
        # 카메라를 설치한 위치와 방향에 맞춰 진행한 좌표계 변환
        x = -round(k * math.cos(alpha_radians), 2) - 345  # 실제 거리에 따라 '345'값을 변경
        y = -(i) - 55    # 실제 거리에 따라 '55'값을 변경
        z = 250
        rx = 177.27
        ry = 0.27
        rz = calculate(x, y)

        coords = [x, y, z, rx, ry, rz]
        print("coords :", coords)
        mc.send_coords(coords, speed, model)  # 로봇을 보정된 좌표로 이동
        wait() # 대기
        current_coords = mc.get_coords()  # 현재 좌표 얻기
        print("current_coords:", current_coords)
        # coords[0] = coords[0] + 25
        # coords[1] = coords[1] + 35
        # #coords[2] = coords[2] + 20 
        coords[2] = 190  # z 좌표 수정
        mc.send_coords(coords, speed, model)  # 로봇을 수정된 좌표로 이동
        wait()  # 대기
        current_coords = mc.get_coords()  # 현재 좌표 얻기
        if current_coords is not None:
            compare_x = abs(current_coords[0] - x)  # x축 비교
            compare_y = abs(current_coords[1] - y)  # y축 비교
            compare_rz = abs(current_coords[5] - rz)  # rz 축 비교
            #print("now_comparing")
            if compare_x < 4 and compare_y < 4 and compare_rz < 4:
                gripper_close()  # 그리퍼 닫기
                wait_gripper()  # 그리퍼 대기
                tracking = False  # 추적 종료
                #print("finish comparing")
                
                return tracking

def calculate(x, y):  # 각도 계산 함수
    x_abs = abs(x)  # x 절대값
    y_abs = abs(y)  # y 절대값
    theta = round(math.degrees(math.atan2(x_abs, y_abs)), 2)  # 각도 계산 및 반올림
    
    if x<0 and y>0:
        return theta
    if x<0 and y<0:
        return 180-theta
#############################################################

##############            MOVE           ###################
#스트림릿으로부터 약 정보를 받은 후, 약통 보관함에서 약을 꺼내 약사에게 약을 전달하는 동작
def wait():  # 대기 함수
    time.sleep(0.6)  # 0.6초 대기
    while mc.is_moving():  # 로봇이 이동 중인 동안
        time.sleep(0.6)  # 0.6초 대기

def wait_gripper():  # 그리퍼 대기 함수
    time.sleep(1.5)  # 0.6초 대기
    while mc.is_gripper_moving():  # 그리퍼가 이동 중인 동안
        time.sleep(1)  # 2초 대기
        
def move_to_start():  # 초기 위치로 이동
    mc.send_angles([0,0,0,0,0,0], speed)  # 각도 전송
    #print("moving_to_start")
    
def move_to_A():  # 약 A 위치로 이동
    mc.send_angles(angles_A_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    mc.send_angles(angles_A, speed)  # 최종 각도 전송
    wait()  # 대기
    #print("moving_to_A")
    gripper_close()  # 그리퍼 닫기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_A_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    move_to_mid_area()  # 중간 위치로 이동
    wait()  # 대기
    move_to_pha()  # 약사 위치로 이동
    wait()  # 대기
    gripper_open()  # 그리퍼 열기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_pha_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    
def move_to_B():  # 약 B 위치로 이동
    mc.send_angles(angles_B_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    mc.send_angles(angles_B, speed)  # 최종 각도 전송
    wait()  # 대기
    #print("moving_to_B")
    gripper_close()  # 그리퍼 닫기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_B_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    move_to_mid_area()  # 중간 위치로 이동
    wait()  # 대기
    move_to_pha()  # 약사 위치로 이동
    wait()  # 대기
    gripper_open()  # 그리퍼 열기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_pha_mid, speed)  # 중간 각도 전송
    wait()  # 대기

def move_to_C():  # 약 C 위치로 이동
    mc.send_angles(angles_C_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    mc.send_angles(angles_C, speed)  # 최종 각도 전송
    wait()  # 대기
    #print("moving_to_C")
    gripper_close()  # 그리퍼 닫기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_C_mid, speed)  # 중간 각도 전송
    wait()  # 대기
    move_to_mid_area()  # 중간 위치로 이동
    wait()  # 대기
    move_to_pha()  # 약사 위치로 이동
    wait()  # 대기
    gripper_open()  # 그리퍼 열기
    wait_gripper()  # 그리퍼 대기
    mc.send_angles(angles_pha_mid, speed)  # 중간 각도 전송
    wait()  # 대기

def move_to_pha():  # 약사 위치로 이동
    mc.send_angles(angles_pha, speed)  # 최종 각도 전송
    wait()  # 대기
    #print("moving_to_pha")
    
def move_to_mid_area():  # 중간 위치로 이동
    mc.send_angles(angles_mid_area, speed)  # 중간 각도 전송
    #print("moving_to_mid_area")
    
def gripper_open():  # 그리퍼 열기
    mc.set_gripper_value(100,20)  # 그리퍼 값 설정
    
def gripper_close():  # 그리퍼 닫기
    mc.set_gripper_value(45,20)  # 그리퍼 값 설정
##########################################################

###############          callback            ###################

def callback_tvec(msg):  # tvec 콜백 함수
    global aruco_X, aruco_Y
    aruco_X = msg.x  # ArUco X 좌표 업데이트
    aruco_Y = msg.y  # ArUco Y 좌표 업데이트
    
def callback_id(msg):  # 마커 ID 콜백 함수
    global marker_id, detecting
    if detecting:
        marker_id = msg.data  # 마커 ID 업데이트
    else:
        marker_id = 0  # 마커 ID 초기화
    
def callback_med(data):  # 약 목록 콜백 함수
    global cleaned_med, med, tvec, to_box_med
    to_box_med = []
    cleaned_med = []
    med = []
    finish_id = []
    med = data.data.split()  # 약 목록 분할
    cleaned_med = [int(item.strip(',')) for item in med]  # 약 목록 정리
    to_box_med = cleaned_med[:]  # 약 목록 복사
    #print("to_box_med :", to_box_med)
    control_mycobot()  # 로봇 제어
    
    #print("finish")
    
def control_mycobot():  # 로봇 제어 함수
    global finish_id, tracking, marker_id, to_box_med, goal_X, goal_Y, detecing
    return_to_ID = 0
    tracking = False

    move_to_start()  # 초기 위치로 이동
    wait()  # 대기
    while cleaned_med:
        move_to_mid_area()  # 중간 위치로 이동
        wait()  # 대기
        
        if cleaned_med[0] == 1:
            move_to_A()  # 약 A 위치로 이동
        if cleaned_med[0] == 2:
            move_to_B()  # 약 B 위치로 이동
        if cleaned_med[0] == 3:
            move_to_C()  # 약 C 위치로 이동
        del cleaned_med[0]  # 약 목록에서 제거
        
        move_to_mid_area()  # 중간 위치로 이동
        wait()  # 대기
        if cleaned_med == None:
            break
            
    move_to_start()  # 초기 위치로 이동
    wait()  # 대기
    
    while to_box_med:
        detecting = True
        while marker_id == 0:
            time.sleep(1)
            if marker_id != 0:
                break
                
        print("marker_id:",marker_id)
        tracking = True
        
        print("aruco_X :", aruco_X, "aruco_Y :", aruco_Y)   
        return_to_ID = marker_id
        detecting = False
        tracking_ArUco(aruco_X, aruco_Y)
        gripper_close()  # 그리퍼 닫기
        wait_gripper()  # 그리퍼 대기
        
        move_to_mid_area()  # 중간 위치로 이동
        wait()  # 대기
        
        if return_to_ID == 1:
            mc.send_angles(angles_A_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            mc.send_angles(angles_A, speed)  # 최종 각도 전송
            wait()  # 대기
            gripper_open()  # 그리퍼 열기
            wait_gripper()  # 그리퍼 대기
            mc.send_angles(angles_A_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            to_box_med.remove(return_to_ID)  # 약 목록에서 제거
            #print("back_to_A :",return_to_ID)
        if return_to_ID == 2:
            mc.send_angles(angles_B_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            mc.send_angles(angles_B, speed)  # 최종 각도 전송
            wait()  # 대기
            gripper_open()  # 그리퍼 열기
            wait_gripper()  # 그리퍼 대기
            mc.send_angles(angles_B_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            to_box_med.remove(return_to_ID)  # 약 목록에서 제거
            #print("back_to_B :",return_to_ID)
        if return_to_ID == 3:
            mc.send_angles(angles_C_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            mc.send_angles(angles_C, speed)  # 최종 각도 전송
            wait()  # 대기
            gripper_open()  # 그리퍼 열기
            wait_gripper()  # 그리퍼 대기
            mc.send_angles(angles_C_mid, speed)  # 중간 각도 전송
            wait()  # 대기
            to_box_med.remove(return_to_ID)  # 약 목록에서 제거
            #print("back_to_C :",return_to_ID)
            
        marker_id = 0  # 마커 ID 초기화
        move_to_mid_area()  # 중간 위치로 이동
        wait()  # 대기
        finish_id.append(return_to_ID)  # 완료된 ID 목록에 추가
        finish_id = sorted(finish_id)  # 완료된 ID 목록 정렬
        #print("to_box_med :", to_box_med)
        
        if to_box_med is None:
            to_box_med = []
            finish_id = []
            marker_id = 0
            break
            
    mc.send_angles([0,0,0,0,0,0],speed)  # 초기 각도로 이동
    marker_id = 0  # 마커 ID 초기화
    
def connect_mycobot():  # MyCobot 연결 함수
    global mc
    port = rospy.get_param("~port", "/dev/ttyACM0")  # 포트 설정
    baud = rospy.get_param("~baud", 115200)  # 보드레이트 설정
    mc = MyCobot(port, baud)  # MyCobot 객체 생성
    
    mc.send_angles([0,0,0,0,0,0],speed)  # 초기 각도로 이동
     
    mc.init_eletric_gripper()  # 전자 그리퍼 초기화
    mc.set_gripper_mode(0)  # 그리퍼 모드 설정
    time.sleep(1)  # 1초 대기
    mc.set_gripper_value(100,30)  # 그리퍼 값 설정
    wait_gripper()  # 그리퍼 대기

    rospy.init_node('mycobot_control', anonymous = True)  # 노드 초기화
    rospy.Subscriber('med_list', String, callback_med)  # 약 목록 구독자 설정
    rospy.Subscriber('/marker_id', Int32, callback_id)  # 마커 ID 구독자 설정
    rospy.Subscriber('/tvec', Vector3, callback_tvec)  # tvec 구독자 설정
    
    rospy.spin()  # ROS 스핀

if __name__ == '__main__':
    try:
        connect_mycobot()  # MyCobot 연결

    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 패스