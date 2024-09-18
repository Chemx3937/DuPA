#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // 명령줄 매개변수를 정수로 변환하기 위해 사용

int main(int argc, char **argv)
{
    // 비디오 소스가 매개변수로 전달되었는지 확인
    if (argv[1] == NULL)
    {
        ROS_INFO("argv[1]=NULL\\n"); // 정보 메시지 출력
        return 1; // 오류 코드 반환
    }

    ros::init(argc, argv, "tracking_image_publisher"); // "tracking_image_publisher"라는 이름으로 ROS 노드 초기화
    ros::NodeHandle nh; // ROS 노드 핸들 생성
    image_transport::ImageTransport it(nh); // 이미지 토픽을 퍼블리시하고 구독하기 위해 ImageTransport 인스턴스 생성
    image_transport::Publisher pub = it.advertise("camera/image", 1); // "camera/image"라는 새로운 이미지 토픽 광고

    ros::Rate loop_rate(5); // 루프 속도를 5Hz로 설정

    // 명령줄 매개변수로 전달된 비디오 장치 인덱스를 정수로 변환
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // 명령줄 매개변수가 숫자인지 확인
    if (!(video_sourceCmd >> video_source))
    {
        ROS_INFO("video_sourceCmd는 %d입니다\\n", video_source); // 정보 메시지 출력
        return 1; // 오류 코드 반환
    }297

    cv::VideoCapture cap(video_source); // 비디오 소스(웹캠) 열기
    // 주어진 인덱스로 비디오 장치를 열 수 있는지 확인
    if (!cap.isOpened())
    {
        ROS_INFO("비디오 장치를 열 수 없습니다\\n"); // 정보 메시지 출력
        return 1; // 오류 코드 반환
    }
    cv::Mat frame; // 비디오 스트림의 각 프레임을 저장할 변수
    sensor_msgs::ImagePtr msg; // ROS 이미지 메시지를 저장할 변수

    while (nh.ok()) // ROS 노드가 실행 중일 때
    {
        cap >> frame; // 비디오 소스로부터 프레임을 읽음
        if (!frame.empty()) // 프레임이 비어 있지 않다면
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); // OpenCV 이미지를 ROS 이미지 메시지로 변환
            pub.publish(msg); // 메시지 퍼블리시
            // ROS_INFO("Published frame at time: %f", ros::Time::now().toSec()); // 프레임 퍼블리시 시간 정보 로그 (필요 시 활성화)
        }
        ros::spinOnce(); // 콜백 함수 호출
        loop_rate.sleep(); // 설정된 주기에 따라 슬립
    }
}