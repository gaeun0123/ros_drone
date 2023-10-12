#!/usr/bin/env python3

import rospy
import astar
import dp
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from dp import douglas_peucker
from drone_config import drone_configs

# 변수 초기화 
current_state = None #드론의 현재 상태를 저장하는 변수
current_local_pose = PoseStamped() #드론의 현재 위치 정보를 저장하는 변수

# 토픽에 메시지가 도착할 때마다 호출되며, 드론의 현재 상태 업데이트
def state_cb(msg):
    global current_state
    current_state = msg

# 토픽에 메시지가 도착할 때마다 호출되며, 드론의 위치 정보 업데이트
def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg

# 거리 계산 함수
def distance(a, b):
    return ((a.pose.position.x - b.position.x)**2 + (a.pose.position.y - b.position.y)**2)**0.5

# ROS 노드 초기화, 주기는 20Hz
rospy.init_node('offboard_node', anonymous=True)

# 다중드론이기 때문에 namespace를 파라미터로 받는다.
namespace = rospy.get_param("~namespace", "").lstrip("/")

print("namespace 받아옴")

rate = rospy.Rate(20)

# mavros/state 토픽 -> state_sub 드론의 상태 정보 제공
# mavros/local_position/pose 토픽 -> local_pose_sub 드론의 위치 정보 제공
# local_pos_pub : 드론에 위치 명령을 보내기 위한 퍼블리셔로, mavros/setpoint_raw/local 토픽에 메시지 전송
state_sub = rospy.Subscriber(namespace + '/mavros/state', State, state_cb)
local_pose_sub = rospy.Subscriber(namespace + '/mavros/local_position/pose', PoseStamped, local_pose_cb)
local_pos_pub = rospy.Publisher(namespace + '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# 드론의 상태 정보가 업데이트 될 때까지 대기
while not rospy.is_shutdown() and current_state is None:
    rate.sleep()

# 목표 위치 선정 
# 처음 위치는 z축만 변환
target_position = PositionTarget()
target_position.position.x = current_local_pose.pose.position.x
target_position.position.y = current_local_pose.pose.position.y
target_position.position.z = current_local_pose.pose.position.z + 0.1
target_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

# 100번동안 목표 위치로 명령 전송
for i in range(100):
    local_pos_pub.publish(target_position)
    rate.sleep()

# 서비스 호출
# set_mode_srv는 드론의 비행 상태를 설정하는 서비스
# arm_srv는 드론의 arm상태를 설정하기 위한 서비스
set_mode_srv = rospy.ServiceProxy(namespace + '/mavros/set_mode', SetMode)
arm_srv = rospy.ServiceProxy(namespace + '/mavros/cmd/arming', CommandBool)

# offboard 모드로 변환 후 드론을 arm 상태로 변환하는 응답
response = set_mode_srv(custom_mode="OFFBOARD")
response = arm_srv(True)

# end_time까지 hovering 명령
hover_duration = rospy.Duration(5)
end_time = rospy.Time.now() + hover_duration
while rospy.Time.now() < end_time:
    local_pos_pub.publish(target_position)
    rate.sleep()

# 현재 위치에서의 상대 고도
desired_altitude = 10  # this altitude is relative to the start altitude, not above sea level

# desired_altitude에 도달할 때까지 0.1m씩 증가하며 드론의 위치 명령 전송
while current_local_pose.pose.position.z < desired_altitude:
    target_position.position.z += 0.1
    local_pos_pub.publish(target_position)
    rate.sleep()

threshold_distance = 0.2 

# A* 알고리즘으로 경로를 가져옴
a = astar.main(namespace)

epsilon = 0.5  # 높은 값은 더 단순한 경로를 생성, 낮은 값은 더 복잡한 경로를 유지

# 알고리즘으로 경로가 생성되지 않을 경우
if a is not None:
    # 단순화 알고리즘 진행
    simplified_path = douglas_peucker(a, epsilon)
else:
    print("error_모든 경로에서 최단 경로가 없습니다.")

for point in a:
    target_position.position.x = point[0]  # x와 y 좌표를 직접 설정
    target_position.position.y = point[1]

    # 도달 판단 기준은 threshold_distance로 정의된 거리 기준
    while distance(current_local_pose, target_position) > threshold_distance:
        local_pos_pub.publish(target_position)
        rate.sleep()

# 드론 자동 착륙 모드 설정
set_mode_srv(custom_mode="AUTO.LAND")
# 노드가 종료될 때까지 대기
rospy.spin()