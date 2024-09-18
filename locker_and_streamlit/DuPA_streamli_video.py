# ros_node.py
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
frame = None

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # 프레임을 파일로 저장하여 Streamlit에서 읽을 수 있게 함
    cv2.imwrite('/tmp/ros_image.jpg', frame)

def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/usb_cam_node/image_raw/compressed', CompressedImage, image_callback)
    # rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, image_callback)
    rospy.spin()
    

if __name__ == '__main__':
    main()