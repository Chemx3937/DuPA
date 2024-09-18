import streamlit as st
import rospy
import cv2
import socket
from std_msgs.msg import String
import datetime
import os
import multiprocessing
import asyncio

# Streamlit 페이지 설정
st.set_page_config(layout="wide")

# 사용자 데이터베이스 예시
user_data = {
    "한기표": "1111",
    "이상희": "1111",
    "손미란": "1111",
    "안체민": "1111",
    "김지웅": "1111",
    "권상혁": "1111",
    "카이로스": "1111",
}

# 로그인 상태를 세션 상태에 저장
if 'logged_in' not in st.session_state:
    st.session_state['logged_in'] = False
if 'username' not in st.session_state:
    st.session_state['username'] = ''

# 로그인 함수
def login(username, password):
    if username in user_data and user_data[username] == password:
        st.session_state['logged_in'] = True
        st.session_state['username'] = username
    else:
        st.error("Invalid username or password")

# 로그아웃 함수
def logout():
    st.session_state['logged_in'] = False
    st.session_state['username'] = ''
    st.experimental_rerun()

# 로그인 페이지
def login_page():
    st.title("Login")
    
    username = st.text_input("Username")
    password = st.text_input("Password", type="password")
    
    if st.button("Login"):
        login(username, password)
        
    if st.session_state['logged_in']:
        st.success("Logged in successfully!")
        st.experimental_rerun()

# 메인 페이지
def main():
    st.sidebar.button("Logout", on_click=logout)

    # 1번째 층 세로 범위 비율(팀 로고, 카메라)
    empty1, con1, empty2 = st.columns([0.2, 1.0, 0.2])
    # 2번째 층 세로 범위 비율(병실, 환자, 약품 선택)
    empty1, con2, empty3, con3, empty2 = st.columns([0.2, 0.5, 0.1, 0.5, 0.2])
    # 3번째 층 세로 범위 비율(로그, 카메라)
    empty1, con4, con5, empty2 = st.columns([0.2, 0.5, 0.5, 0.2])

    # 호실, 환자, 처방 약품 리스트
    selected_room = None
    selected_patient = None
    med_list = []

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
    shared_data['step'] = "None"
    shared_data['log'] = "None"

    # 콜백 함수 정의
    def step_callback(data):
        shared_data['step'] = data.data

    def log_callback(data):
        shared_data['log'] = data.data

    def ros_process(shared_data):
        # ROS 노드 초기화
        rospy.init_node('subscriber_app', anonymous=True)
        rospy.Subscriber('step', String, step_callback)
        rospy.Subscriber('send_log', String, log_callback)
        rospy.spin()

    # ROS 프로세스를 시작합니다
    ros_p = multiprocessing.Process(target=ros_process, args=(shared_data,))
    ros_p.start()

    def get_step_image(step_number):
        return f"/home/hui/myagv_ros/src/myagv_navigation/step/{step_number}.png"

    async def update_camera(frame_placeholder):
        while True:
            if os.path.exists('/tmp/ros_image.jpg'):
                frame = cv2.imread('/tmp/ros_image.jpg')
                if frame is not None:
                    frame_placeholder.image(frame, channels="BGR", width=500)
            await asyncio.sleep(0.1)

    async def update_steps(path_image_placeholder):
        while True:
            step = shared_data['step']
            if step and step != "None":
                step_image_path = get_step_image(step)
                if os.path.exists(step_image_path):
                    path_image_placeholder.image(step_image_path, width=500)
                await asyncio.sleep(1)
                path_image_placeholder.image('/home/hui/myagv_ros/src/myagv_navigation/room/room0.png', width=500)
                await asyncio.sleep(1)
            await asyncio.sleep(0.1)

    async def update_logs(log_placeholder):
        while True:
            log = shared_data['log']
            log_placeholder.markdown(f"<h1 style='font-size:24px; text-align:left;'>{log}</h1>", unsafe_allow_html=True)
            await asyncio.sleep(0.1)

    with empty1:
        st.empty()  # 여백부분1

    ### 1번째 층
    with con1:
        st.image('/home/hui/myagv_ros/src/myagv_navigation/dupa.png', width=250)
        st.write("Doctor, Nurse, Pharmacy Assistants")

    ### 2번째 층
    with con2:
        room_expander = st.expander("병실 선택")
        with room_expander:
            selected_room = st.selectbox('병실', ['select', 'room1', 'room2'])

        patient_expander = st.expander("환자 선택")
        with patient_expander:
            selected_patient = st.selectbox('환자번호', ['select', 'patient1', 'patient2', 'patient3', 'patient4'])

        med_expander = st.expander("처방 약품 선택")
        with med_expander:
            if st.checkbox("덱사메타손", key="1_button"):
                if "1" not in med_list:
                    med_list.append("1")
            if st.checkbox("지르텍 정", key="2_button"):
                if "2" not in med_list:
                    med_list.append("2")
            if st.checkbox("프레드니손", key="3_button"):
                if "3" not in med_list:
                    med_list.append("3")
            if st.checkbox("이부프로펜", key="4_button"):
                if "4" not in med_list:
                    med_list.append("4")
            if st.checkbox("아지트로마이신", key="5_button"):
                if "5" not in med_list:
                    med_list.append("5")
            if st.checkbox("아빌리파이 정", key="6_button"):
                if "6" not in med_list:
                    med_list.append("6")

    with con3:
        data = {
            "항목": ["병실", "환자 번호", "처방 약품"],
            "선택": [selected_room, selected_patient, ', '.join(med_list)]
        }
        st.table(data)

        check = st.radio('처방 약품을 맞게 하셨나요?', ['Not yet', 'Yes', 'Open', 'Return'])
        if check == 'Yes':
            send_to_ros('room', selected_room)
            send_to_ros('patient', selected_patient)
            send_to_ros('med_list', ', '.join(med_list))
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if 'log' not in st.session_state:
                st.session_state.log = []
            st.session_state.log.append(f"** {current_time} **")
            st.session_state.log.append(f">>> {selected_room}으로 {selected_patient}을 위한 {med_list}를 요청하였습니다.")
            
        elif check == 'Open':
            send_to_ros('open', 'open')
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if 'log' not in st.session_state:
                st.session_state.log = []
            st.session_state.log.append(f"** {current_time} **")
            st.session_state.log.append(f">>> {selected_room}에서 약품 보관함이 열렸습니다.")
                     
        elif check == 'Return':
            send_to_ros('return', 'return')
            current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if 'log' not in st.session_state:
                st.session_state.log = []
            st.session_state.log.append(f"** {current_time} **")
            st.session_state.log.append(f">>> {selected_room}에서 {selected_patient}을 위한 {med_list}를 수령했습니다.")

    ### 3번째 층
    with con4:
        if 'log' not in st.session_state:
            st.session_state.log = []

        if st.session_state.log:
            for log_message in st.session_state.log:
                st.text(log_message)

    with con5:
        st.header("Camera and Map")
        frame_placeholder = st.empty()
        path_image_placeholder = st.image('/home/hui/myagv_ros/src/myagv_navigation/room/room0.png', width=500)
        log_placeholder = st.empty()

    async def main_async():
        await asyncio.gather(update_camera(frame_placeholder), update_steps(path_image_placeholder), update_logs(log_placeholder))

    asyncio.run(main_async())

if __name__ == "__main__":
    if st.session_state['logged_in']:
        main()
    else:
        login_page()
