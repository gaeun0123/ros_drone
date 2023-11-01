#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def own_pose_callback(data):
    # MAVROS로부터 로컬 좌표 정보를 받으면 공유 토픽에 그대로 발행
    local_pose_publisher.publish(data)

# ROS 노드 초기화, 해당 노드를 식별하기 위한 명 : drone1_local_pose_publishing
rospy.init_node('drone1_local_pose_publishing')

# MAVROS의 로컬 좌표 정보 토픽을 구독
rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, own_pose_callback)

# 로컬 좌표 정보를 발행하기 위한 Publisher
local_pose_publisher = rospy.Publisher("/drone1/local_pose", PoseStamped, queue_size=10)

rospy.spin()

# drone1의 로컬 위치를 받아 /drone1/local_pose 토픽으로 발행하는 코드