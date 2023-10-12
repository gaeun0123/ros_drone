#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def own_pose_callback(data):
    # MAVROS로부터 위치 정보를 받으면 공유 토픽에 그대로 발행
    shared_pose_publisher.publish(data)

# ROS 노드 초기화, 해당 노드를 식별하기 위한 명 : drone1_pose_sharing
rospy.init_node('drone1_pose_sharing')

# MAVROS의 위치 정보 토픽을 구독
rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, own_pose_callback)

# 공유 토픽에 위치 정보를 발행하기 위한 Publisher
shared_pose_publisher = rospy.Publisher("/drone1/shared_pose", PoseStamped, queue_size=10)

rospy.spin()

# 결론적으로 이 코드는
# drone1_pose_sharing이라는 노드가
# /mavros/local_position/pose에서 위치 정보를 받아
# /drone1/shared_pose에 재발행하는 역할을 맡는다.
# 따라서 /drone1/share_pose 토픽을 구독하면 드론1의 위치를 얻을 수 있다.