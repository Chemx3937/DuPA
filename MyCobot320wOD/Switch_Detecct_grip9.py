from pymycobot.mycobot import MyCobot
import time
import cv2
from ultralytics import YOLO
import numpy as np
import math 

# YOLOv8 모델 로드
model = YOLO('best.pt')  # 사용할 모델로 변경

# Joint 초기각도 및 목표 각도 설정
angles0 = [0, 0, 0, 0, 0, 0]
angles_1 = [-71.54, -37.79, 5.97, -57.65, 92.9, -159]
angles_2 = [-72.77, -109.95, 87.27, -65, 87.45, -157]
angles_3 = [-116.27, -69.87, 45.61, -63.63, 90.79, -157]
angles_4 = [-115.48, -101.58, 66.35, -50, 89.38, -21]

# Gripper 초기값 및 잡았을 때 값 설정
g_open = 100
g_close = 15

# 로봇암 움직이기 시작
mc = MyCobot('COM3', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(2)

# Angle0로 이동, Gripper = g_open
mc.send_angles(angles0, 20)
mc.set_gripper_value(100, 20)
print(f'시작, Joint: Angle0({angles0}), Gripper: {g_open}')
time.sleep(2)

# 웹캠 설정
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # 프레임의 너비, 가로 픽셀 수
cap.set(4, 480)  # 프레임의 높이, 세로 필셀 수
print('카메라: ON')

# 프레임 처리 및 객체 인식 시작
while True:
    # 'esc' 키 감지시 Angle0로 이동
    if cv2.waitKey(1) & 0xFF == 27:
        mc.send_angles(angles0, 20)
        mc.set_gripper_value(100, 20)
        print(f'ESC눌림 -> 복귀, Joint: Angle0({angles0}), Gripper: {g_open}')
        break
    
    # Angle1로 이동, Gripper = g_open
    mc.send_angles(angles_1, 10)
    print(f'Joint: Angle1({angles_1}), Gripper: {g_open}')
    print('스위치의 색을 확인합니다.')
    time.sleep(10)

    # 카메라로부터 프레임을 가져와 객체 인식
    object_recognized = False
    while not object_recognized and cap.isOpened():
        success, frame = cap.read()
        if not success:
            print('인식하지 못했습니다.')
            continue

        # YOLOv8 추론 실행 및 결과 시각화
        results = model(frame)
        annotated_frame = results[0].plot()

        # 프레임 크기 조정 (예: 너비 1280, 높이 720)
        resized_frame = cv2.resize(annotated_frame, (1280, 720))

        # 조정된 크기의 프레임 표시
        cv2.imshow("YOLOv8 추론", resized_frame)

        for r in results:
            boxes = r.boxes
            if len(boxes) > 0:
                cls_np = boxes.cls.numpy()
                object = cls_np[0]

                if object == 0:  # 빨간색
                    object_recognized = True
                    print("빨간색 스위치가 인식됐습니다.")

                    # Angle2로 이동 및 색상 감지 시작, Gripper = g_open
                    mc.send_angles(angles_2, 10)
                    print(f'Joint: Angle2({angles_2}), Gripper: {g_open}')
                    time.sleep(10)
                    
                    # Angle2에서 빨간색 인식 후 스위치 잡기, Gripper = g_close
                    mc.set_gripper_value(15, 20)
                    print(f'Joint: Angle2({angles_2}), Gripper: {g_close}')
                    time.sleep(5)

                    # Angle3로 이동, Gripper = g_close
                    mc.send_angles(angles_3, 10)
                    print(f'Joint: Angle3({angles_3}), Gripper: {g_close}')
                    time.sleep(10)

                    # Angle4로 이동 후 스위치 내려놓기, Gripper = g_open
                    mc.send_angles(angles_4, 10)
                    time.sleep(10)
                    mc.set_gripper_value(100, 20)
                    print(f'Joint: Angle4({angles_4}), Gripper: {g_open}')
                    time.sleep(5)

                    # Angle0로 복귀, Gripper = g_open
                    mc.send_angles(angles0, 20)
                    mc.set_gripper_value(100, 20)
                    print(f'복귀, Joint: Angle0({angles0}), Gripper: {g_open}')
                    print('빨간색 스위치 Logic 완료')
                    time.sleep(10)
                    break

                elif object == 1:  # 초록색
                    object_recognized = True
                    print("초록색 스위치가 인식됐습니다. Angle0로 복귀합니다.")

                    # Angle0로 복귀, Gripper = g_open
                    mc.send_angles(angles0, 20)
                    mc.set_gripper_value(100, 20)
                    print(f'복귀, Joint: Angle0({angles0}), Gripper: {g_open}')
                    print('초록색 스위치 Logic 완료')
                    time.sleep(10)
                    break

# 정리
cv2.destroyAllWindows()
cap.release()