#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
import tf
from tf.broadcaster import TransformBroadcaster
import tf_conversions
import yaml

class ImageConverter:
    def __init__(self):
        self.br = TransformBroadcaster()  # TransformBroadcaster 인스턴스 생성
        self.bridge = CvBridge()  # CvBridge 인스턴스 생성
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)  # ArUco 마커 사전 설정, 6x6을 생성한 마커의 Dictionary에 맞게 수정
        self.aruo_params = cv.aruco.DetectorParameters_create()  # ArUco 마커 탐지 매개변수 생성
        
        # YAML 파일에서 카메라 매트릭스와 왜곡 계수 읽기
        calibration_file = rospy.get_param("~calibration_file", "camera.yaml")
        with open(calibration_file, "r") as file:
            calib_data = yaml.safe_load(file)
            self.camera_matrix = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))
            self.dist_coeffs = np.array(calib_data["distortion_coefficients"]["data"])
        
        rospy.loginfo(f"Camera Matrix: \\n{self.camera_matrix}")
        rospy.loginfo(f"Distortion Coefficients: \\n{self.dist_coeffs}")

        self.current_marker_id = None  # 현재 반환 중인 마커 ID 저장
        self.marker_detected_time = None  # 마커가 처음 인식된 시간을 저장

        # "/camera/image" 토픽으로부터 이미지를 구독
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback, queue_size=1, buff_size=2**24)
        # '/tvec' 토픽 퍼블리셔 생성
        self.tvec_pub = rospy.Publisher('/tvec', Vector3, queue_size=10)
        # '/marker_id' 토픽 퍼블리셔 생성
        self.marker_id_pub = rospy.Publisher('/marker_id', Int32, queue_size=10)

    def callback(self, data):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {0}".format(e))
            return

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)  # 이미지를 그레이스케일로 변환
        ret = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruo_params)  # ArUco 마커 탐지
        corners, ids = ret[0], ret[1]  # 마커의 코너와 ID를 가져옴

        if len(corners) > 0 and ids is not None:  # 마커가 탐지된 경우
            if self.current_marker_id is None or self.current_marker_id not in ids:
                self.current_marker_id = ids[0][0]  # 현재 마커 ID 설정
                self.marker_detected_time = rospy.Time.now()  # 마커가 처음 인식된 시간 설정

            marker_index = np.where(ids == self.current_marker_id)[0][0]  # 현재 마커 ID의 인덱스 찾기
            ret = cv.aruco.estimatePoseSingleMarkers(
                [corners[marker_index]], 0.02, self.camera_matrix, self.dist_coeffs
            )  # 마커의 자세 추정
            (rvec, tvec) = (ret[0], ret[1])
            tvec = tvec * 1000  # tvec 값을 mm 단위로 변환
            tvec = np.round(tvec, 2)  # 소수점 둘째 자리까지 반올림

            tvec_msg = Vector3(x=tvec[0, 0, 0], y=tvec[0, 0, 1], z=tvec[0, 0, 2])  # tvec 값을 메시지로 변환
            current_time = rospy.Time.now()  # 현재 시간 저장

            if self.marker_detected_time and (current_time - self.marker_detected_time).to_sec() >= 3:
                # 마커가 3초 이상 인식되었을 때 퍼블리시
                rospy.loginfo("Publishing marker ID: {0}, tvec: {1}".format(self.current_marker_id, tvec_msg))
                self.tvec_pub.publish(tvec_msg)  # tvec 퍼블리시
                self.marker_id_pub.publish(Int32(self.current_marker_id))  # 마커 ID 퍼블리시

            cv.aruco.drawDetectedMarkers(cv_image, [corners[marker_index]])  # 인식된 마커 그리기
            cv.aruco.drawAxis(
                cv_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec[0, :, :],
                tvec[0, :, :],
                0.03,
            )  # 마커의 좌표축 그리기

            xyz = tvec[0, 0, :]
            xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.03]  # 좌표 변환

            euler = rvec[0, 0, :]
            tf_change = tf.transformations.quaternion_from_euler(
                euler[0], euler[1], euler[2]
            )  # 회전 변환

            self.br.sendTransform(
                xyz, tf_change, rospy.Time.now(), "basic_shapes", "joint6_flange"
            )  # TF 브로드캐스트
        else:
            self.current_marker_id = None  # 마커가 인식되지 않으면 초기화
            self.marker_detected_time = None

        # 이미지 디스플레이
        cv.imshow("Image", cv_image)
        if cv.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User requested shutdown")  # 사용자가 'q'를 눌러 노드를 종료 요청

    def start(self):
        rospy.loginfo("Starting image converter node")  # 노드 시작 로그 출력
        rospy.spin()  # ROS 콜백 루프 시작

if __name__ == "__main__":
    try:
        rospy.init_node("detect_marker")  # "detect_marker"라는 이름으로 ROS 노드 초기화
        node = ImageConverter()  # ImageConverter 인스턴스 생성
        node.start()  # 노드 시작
    except KeyboardInterrupt:
        print("Shutting down detect_marker node.")  # 키보드 인터럽트 시 메시지 출력
        cv.destroyAllWindows()  # 모든 OpenCV 윈도우 닫기