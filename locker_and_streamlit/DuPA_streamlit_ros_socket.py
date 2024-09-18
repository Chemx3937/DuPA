#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket

def start_server():
    host = 'localhost'
    port = 12345
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("Server is waiting for client request...")

    pub_room = rospy.Publisher('selected_room', String, queue_size=10)
    pub_patient = rospy.Publisher('selected_patient', String, queue_size=10)
    pub_med_list = rospy.Publisher('med_list', String, queue_size=10)
    pub_start = rospy.Publisher('start', String, queue_size=10)
    pub_open = rospy.Publisher('open', String, queue_size=10)
    pub_return = rospy.Publisher('return', String, queue_size=10)
    rospy.init_node('publisher_app', anonymous=True)

    while not rospy.is_shutdown():
        conn, addr = server_socket.accept()
        print(f"Connected to {addr}")
        while not rospy.is_shutdown():
            data = conn.recv(1024).decode('utf-8')
            if not data:
                break
            rospy.loginfo(f"Received data: {data}")

            # 데이터의 타입을 구분하여 각각의 주제로 퍼블리시
            if data.startswith('room:'):
                room_data = data.replace('room:', '')
                rospy.loginfo(f"Publishing {room_data} to selected_room topic")
                pub_room.publish(room_data)
            elif data.startswith('patient:'):
                patient_data = data.replace('patient:', '')
                rospy.loginfo(f"Publishing {patient_data} to selected_patient topic")
                pub_patient.publish(patient_data)
            elif data.startswith('med_list:'):
                med_list_data = data.replace('med_list:', '')
                rospy.loginfo(f"Publishing {med_list_data} to med_list topic")
                pub_med_list.publish(med_list_data)
            elif data.startswith('start:'):
                start_data = data.replace('start:', '')
                rospy.loginfo(f"Publishing {start_data} to start topic")
                pub_start.publish(start_data)
            elif data.startswith('open'):
                open_data = data.replace('open:', '' )
                rospy.loginfo(f"Publishing {open_data} to open topic")
                pub_open.publish(open_data)
            elif data.startswith('return'):
                return_data = data.replace('return:', '' )
                rospy.loginfo(f"Publishing {return_data} to return topic")
                pub_return.publish(return_data)
        conn.close()

if __name__ == "__main__":
    start_server()
