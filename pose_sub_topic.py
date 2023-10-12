#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import astar

def drone1_pose_callback(data):
    # Data 객체에서 위치 정보를 얻어 문자열로 변환
    global drone1_pose
    drone1_pose = data.pose
    # position_str = "x: {}, y: {}, z: {}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z)
    # print("drone2 : drone1과 송신되었으며, drone1의 위치는 " + position_str)

rospy.init_node('drone2_pose_listener')

# drone1의 공유 토픽을 구독
rospy.Subscriber("/drone1/shared_pose", PoseStamped, drone1_pose_callback)

# 토픽을 받으면 drone1_pose_callback함수 실행
rospy.spin()
