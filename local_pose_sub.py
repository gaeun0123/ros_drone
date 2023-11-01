#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def drone1_local_pose_callback(data):
    # /drone1/local_pose 토픽으로부터 drone1의 로컬 좌표 정보
    rospy.loginfo(f"drone2 : Drone1의 local 위치 : {data.pose}")

# ROS 노드 초기화, 해당 노드를 식별하기 위한 이름: drone2_receive_pose
rospy.init_node('drone2_receive_pose')

# drone1의 로컬 좌표 정보 토픽을 구독
rospy.Subscriber("/drone1/local_pose", PoseStamped, drone1_local_pose_callback)

rospy.spin()

# drone2가 /drone1/local_pose 토픽을 구독하여 data를 얻는 코드