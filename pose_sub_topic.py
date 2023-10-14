#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def drone1_pose_callback(data):
    # Data 객체에서 위치 정보를 얻어 문자열로 변환
    global drone1_pose
    drone1_pose = {
    "latitude": data.latitude,
    "longitude": data.longitude,
    "altitude": data.altitude
    }
    position_str = "latitude : {}, longitude : {}, altitude : {}".format(drone1_pose["latitude"], drone1_pose["longitude"], drone1_pose["altitude"])
    print("drone2 : drone1과 통신되었으며, drone1의 위치 " + position_str)

rospy.init_node('drone2_pose_listener')

# drone1의 공유 토픽을 구독
rospy.Subscriber("/drone1/shared_pose", NavSatFix, drone1_pose_callback)

# 토픽을 받으면 drone1_pose_callback함수 실행
rospy.spin()
