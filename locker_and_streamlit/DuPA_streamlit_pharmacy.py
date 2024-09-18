import streamlit as st
import rospy
from std_msgs.msg import String
import threading
import multiprocessing
import time
import os
import socket

# Streamlit 페이지 설정
st.set_page_config(layout="wide")

# 1번째 층 세로 범위 비율(팀 로고, 제목)
empty1, con1, empty2, con2, empty2 = st.columns([0.3, 0.3, 0.05, 0.7, 0.3])
# 2번째 층 세로 범위 비율(병실, 환자, 약품 선택)
empty1, con3, empty3, con4, empty2 = st.columns([0.3, 0.5, 0.1, 0.5, 0.3])
# 3번째 층 세로 범위 비율(로그, 카메라)
empty1, con5, empty2 = st.columns([0.3, 1.0, 0.3])
# 4번째 층 세로 범위 비율(리셋 버튼)
empty1, con6, empty2 = st.columns([0.3, 1.0, 0.3])
# 5번째 층 세로 범위 비율(AGV출발 버튼)
empty1, con7, empty2 = st.columns([0.3, 1.0, 0.3])

selected_start = 'Start'

def send_to_ros(tag, message):
    host = 'localhost'
    port = 12345
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client_socket.connect((host, port))
        client_socket.sendall(f"{tag}:{message}".encode('utf-8'))
        st.success(f"{message}를(을) {tag}로 전송했습니다.")
    except Exception as e:
        st.error(f"서버와 연결하는 데 실패했습니다: {e}")
    finally:
        client_socket.close()

# 콜백 함수에서 접근할 수 있도록 매니저 딕셔너리를 초기화
manager = multiprocessing.Manager()
shared_data = manager.dict()
shared_data['room'] = "None"
shared_data['patient'] = "None"
shared_data['med_list'] = "None"

# 약품 정보와 사진을 저장하는 딕셔너리
med_info_dict = {
    "1": {"name": "덱사메타손 (Dexamethasone)", "image": "1", "info": "염증성 질환, 알레르기 반응, 자가면역 질환 등을 치료하는 스테로이드성 약품입니다."},
    "2": {"name": "지르텍 정 (Zyrtec)", "image": "2", "info": "알레르기성 비염, 두드러기, 피부 가려움증을 치료하는 알레르기 치료 약품입니다."},
    "3": {"name": "프레드니손 (Prednisone)", "image": "3", "info": "염증성 질환, 알레르기 반응, 자가면역 질환 등을 치료하는 스테로이드성 약품입니다."},
    "4": {"name": "이부프로펜 (Ibuprofen)", "image": "4", "info": "진통, 해열, 항염증 작용을 하는 진통제 입니다."},
    "5": {"name": "아지트로마이신 (Azithromycin)", "image": "5", "info": "기관지염, 폐렴, 부비동염 등을 치료하는 항생제 입니다."},
    "6": {"name": "아빌리파이 정 (Abilify)", "image": "6", "info": "정신 장애 증상을 치료하는 약품입니다."}
}

# 콜백 함수 정의
def room_callback(data):
    shared_data['room'] = data.data

def patient_callback(data):
    shared_data['patient'] = data.data

def med_list_callback(data):
    shared_data['med_list'] = data.data

def ros_process(shared_data):
    # ROS 노드 초기화
    rospy.init_node('streamlit_medicine', anonymous=True)
    rospy.Subscriber('selected_room', String, room_callback)
    rospy.Subscriber('selected_patient', String, patient_callback)
    rospy.Subscriber('med_list', String, med_list_callback)
    rospy.spin()

# ROS 프로세스를 시작합니다
ros_p = multiprocessing.Process(target=ros_process, args=(shared_data,))
ros_p.start()

def get_patient_image(patient_name):
    # 실제 환자 이미지 경로로 변경 필요
    return f"/home/hui/myagv_ros/src/myagv_navigation/patient_images/{patient_name}.png"

def get_med_image(med_name):
    # 실제 약품 이미지 경로로 변경 필요
    return f"/home/hui/myagv_ros/src/myagv_navigation/med_images/{med_name}.png"

def get_room_image(room_name):
    return f"/home/hui/myagv_ros/src/myagv_navigation/room/{room_name}.png"

def main():
    with empty1:
        st.empty()  # 여백부분1
        
    with empty3:
        st.empty()

    ### 1번째 층
    with con1:
        ## 팀 로고 이미지
        st.image('/home/hui/myagv_ros/src/myagv_navigation/dupa.png', width=250)
        st.markdown("<p>Doctor, Nurse, Pharmacy Assistants</p>", unsafe_allow_html=True)

    with con2:
        st.markdown("<h1 style='text-align: left; font-size: 55px;'>DuPA 환자 약품 확인서(조제실)</h1>", unsafe_allow_html=True)

    ### 2번째 층
    with con3:
        patient_info = st.expander('환자 정보')
        with patient_info:
            patient_image_placeholder = st.image('/home/hui/myagv_ros/src/myagv_navigation/med_images/0.png', width=250)
            patient_placeholder = st.empty()
            room_placeholder = st.empty()

    with con4:
        room_image_placeholder = st.image('/home/hui/myagv_ros/src/myagv_navigation/room/room0.png', width=500)
    
    ### 3번째 층
    with con5:
        med_info = st.expander('약품 정보')
        med_placeholder = st.empty()  # 약품 정보를 표시할 공간

    ### 4번째 층
    with con6:
        reset_button_placeholder = st.empty()  # 리셋 버튼을 표시할 공간

    ### 5번째 층
    with con7:
        agv_button_placeholder = st.empty()  # AGV 출발 버튼을 표시할 공간

    # AGV 출발 버튼 표시
    with con7:
        if st.button("AGV 출발"):
            
            send_to_ros('start', selected_start)
            shared_data['patient'] = "None"
            shared_data['room'] = "None"
            shared_data['med_list'] = "None"
            patient_placeholder.empty()
            room_placeholder.empty()
            med_placeholder.empty()
            patient_image_placeholder.image('/home/hui/myagv_ros/src/myagv_navigation/med_images/0.png', width=250)
            room_image_placeholder.image('/home/hui/myagv_ros/src/myagv_navigation/room/room0.png', width=500)

    while True:
        patient_name = shared_data['patient']
        room = shared_data['room']
        med_list = shared_data['med_list'].split(", ")

        patient_placeholder.markdown(f"<h1 style='font-size:24px; text-align:left;'>환자 번호 : {patient_name}</h1>", unsafe_allow_html=True)        
        room_placeholder.markdown(f"<h1 style='font-size:24px; text-align:left;'>병실 호수 : {room}</h1>", unsafe_allow_html=True)

        # 환자 이미지 표시
        if patient_name and patient_name != "None":
            patient_image_path = get_patient_image(patient_name)
            if os.path.exists(patient_image_path):
                patient_image_placeholder.image(patient_image_path, width = 250)
                
        if room and room != "None":
            room_image_path = get_room_image(room)
            if os.path.exists(room_image_path):
                room_image_placeholder.image(room_image_path, width = 500)
        
        # 약품 정보 표시
        med_placeholder.empty()
        with med_placeholder.container():
            for med_id in med_list:
                if med_id in med_info_dict:
                    med_info = med_info_dict[med_id]
                    col1, col2 = st.columns([0.2, 0.8])
                    with col1:
                        med_image_path = get_med_image(med_info["image"])
                        if os.path.exists(med_image_path):
                            st.image(med_image_path, width=200)
                    with col2:
                        st.markdown(f"**{med_info['name']}**")
                        st.markdown(med_info['info'])
                st.markdown("---")
        
        time.sleep(1)  # 1초마다 업데이트

if __name__ == "__main__":
    main()

    # 프로세스 종료를 위해 Streamlit이 종료될 때 호출
    ros_p.join()
