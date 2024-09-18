#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import time

done_detected = False
start_detected = False
open_detected = False
return_detected = False
room_number = 0

        
def command_done_callback(data):
    
    global done_detected
    
    if data.data == 'done': 
        done_detected = True
        
def command_start_callback(data):
    
    global start_detected
    
    if data.data == 'Start': 
        start_detected = True

def command_open_callback(data):
    
    global open_detected
    
    if data.data == 'open': 
        open_detected = True

def command_return_callback(data):
    
    global return_detected
    
    if data.data == 'return': 
        return_detected = True

def room_callback(data):
    rospy.loginfo("Selected room: %s", data.data)
    
    global room_number
    
    
    if data.data == 'room1':
        room_number = 1
    elif data.data == 'room2':
        room_number = 2
    else:
        rospy.loginfo("Invalid room selection")

def main():
    global done_detected
    global room_number  
    global start_detected
    global open_detected
    global return_detected
    
    rospy.init_node('user_input_publisher', anonymous=True)
    pub = rospy.Publisher('user_destination', String, queue_size=10)
    pub_log = rospy.Publisher('send_log', String, queue_size=10)
    pub_step = rospy.Publisher('step', String, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    
    rospy.Subscriber('move_base_done', String, command_done_callback)
    rospy.Subscriber('selected_room', String, room_callback)
    rospy.Subscriber('start', String, command_start_callback)
    rospy.Subscriber('open', String, command_open_callback)
    rospy.Subscriber('return', String, command_return_callback)

    case = 0

    while not rospy.is_shutdown():

        # 병실 선택
        if case == 0 :
            time.sleep(1)
            pub_log.publish("*** DuPA 약품 운송 로봇 대기 중입니다. ***")
            pub_step.publish('0')
            if room_number == 1:
                case += 100
            elif room_number == 2:
                case += 200
            else :
                pass
            
        # 병실 1
        elif case == 100:                   
            pub.publish('3')
            pub_step.publish('100')
            rospy.loginfo("Sent destination: 3")
            pub_log.publish("*** 조제실로 이동 중 입니다. ***")

            if done_detected == True:
                pub_log.publish("*** 조제실에 도착 했습니다. ***")
                case += 5
                
                
        elif case == 105:
            done_detected = False
            pub_log.publish("*** 약품을 수령 중 입니다. ***")
            pub_step.publish('105')
            rospy.loginfo("Waiting Storage")
            
            if start_detected == True:
                pub_log.publish("*** 약품을 수령 했습니다. ***")
                pub_log.publish("*** 1번 병실로 출발합니다. ***")
                case += 3
        
        elif case == 108:
            start_detected = False
            pub_step.publish('108')
            pub_log.publish("*** 1번 병실로 출발합니다. ***")
            time.sleep(2) # 서보모터 작동 시간
            case += 2
            
                              
        elif case == 110 :
            start_detected = False
            time.sleep(0.5)            
            pub.publish('1')
            pub_step.publish('110')
            pub_log.publish("*** 1번 병실로 이동 중 입니다. ***")
            rospy.loginfo("Sent destination: 1")
            if done_detected == True:
                pub_log.publish("*** 1번 병실에 도착 했습니다. ***")
                case += 5
                
        elif case == 115 :
            done_detected = False
            time.sleep(0.5)   
            pub_step.publish('115')
            pub_log.publish("*** 간호사가 약품을 수령 중 입니다. ***")
            if return_detected == True:
                pub_log.publish("*** 1번 병실에서 약품을 수령 했습니다. ***")
                pub_log.publish("*** NS로 복귀합니다. ***")
                case += 5        

        elif case == 120:
            return_detected = False
            pub_log.publish("*** NS로 복귀합니다. ***")
            time.sleep(0.5)            
            pub.publish('4')
            pub_step.publish('120')
            pub_log.publish("*** NS로 복귀 중 입니다. ***")
            rospy.loginfo("Sent destination: 4")
            if done_detected == True:
                pub_log.publish("*** NS로 복귀 했습니다. ***")
                pub_log.publish("*** 안내를 종료합니다. ***")
                room_number = 0
                case -= 120
                        
        
        # 병실 2
        elif case == 200:                   
            pub.publish('3')
            pub_step.publish('200')
            rospy.loginfo("Sent destination: 3")
            pub_log.publish("*** 조제실로 이동 중 입니다. ***")

            if done_detected == True:
                pub_log.publish("*** 조제실에 도착 했습니다. ***")
                case += 5
                
                
        elif case == 205:
            done_detected = False
            pub_step.publish('205')
            pub_log.publish("*** 약품을 수령 중 입니다. ***")
            rospy.loginfo("Waiting Storage")
            
            if start_detected == True:
                pub_log.publish("*** 약품을 수령 했습니다. ***")
                pub_log.publish("*** 2번 병실로 출발합니다. ***")
                case += 3
        
        elif case == 208:
            start_detected = False
            pub_step.publish('208')
            pub_log.publish("*** 2번 병실로 출발합니다. ***")
            time.sleep(2) # 서보모터 작동 시간
            case += 2
            
                              
        elif case == 210 :
            start_detected = False
            time.sleep(0.5)            
            pub.publish('2')
            pub_step.publish('210')
            pub_log.publish("*** 2번 병실로 이동 중 입니다. ***")
            rospy.loginfo("Sent destination: 2")
            if done_detected == True:
                pub_log.publish("*** 2번 병실에 도착 했습니다. ***")
                case += 5
                
        elif case == 215 :
            done_detected = False
            time.sleep(0.5)   
            pub_step.publish('215')
            pub_log.publish("*** 간호사가 약품을 수령 중 입니다. ***")
            if return_detected == True:
                pub_log.publish("*** 2번 병실에서 약품을 수령 했습니다. ***")
                pub_log.publish("*** NS로 복귀합니다. ***")
                case += 5        

        elif case == 220:
            return_detected = False
            time.sleep(0.5)            
            pub.publish('4')
            pub_step.publish('220')
            pub_log.publish("*** NS로 복귀 중 입니다. ***")
            rospy.loginfo("Sent destination: 4")
            if done_detected == True:
                pub_log.publish("*** NS로 복귀 했습니다. ***")
                pub_log.publish("*** 안내를 종료합니다. ***")
                room_number = 0
                case -= 220           
               

        else:
            rospy.loginfo("Invalid choice. Please enter a valid option.")

        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass