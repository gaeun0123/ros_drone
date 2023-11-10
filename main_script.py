#!/usr/bin/env python3

import rospy
from modified_astar import astar as cal_path
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from dp import douglas_peucker
import drone_config
from geometry_msgs.msg import Pose, Point
from modified_astar import calculate_distance as cal_dis
from modified_astar import recalculation_path as recal_path
from nav_msgs.msg import Path

path = []
current_state = None
current_local_pose = PoseStamped()
global drone_positions
drone_positions = {}
CRITICAL = 5.0

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg
    
def distance2(a, b):
    return ((a.pose.position.x - b.position.x)**2 + (a.pose.position.y - b.position.y)**2)**0.5

def position_callback(msg, drone_name):
    global drone_positions, current_local_pose, path

    # # 통신 : 다른 드론 위치 데이터
    # rospy.loginfo(f"Received {drone_name} position: {msg.pose.position}")

    # # 드론 위치 데이터
    # rospy.loginfo(f"Current local pose: {current_local_pose.pose.position}")

    # drone_name = 'uav0'으로 지정, 의 위치 정보 msg.pose.position 저장
    drone_positions[drone_name] = msg.pose.position

    if current_local_pose is not None and drone_name in drone_positions:
        other_position = drone_positions[drone_name]

        distance = cal_dis(current_local_pose.pose.position, other_position)

        # distance가 임계값 이하일 경우, 경로 재계산
        if distance < CRITICAL:
            rospy.loginfo(f"{current_local_pose.pose.position}")
            rospy.loginfo(f"{other_position}")
            rospy.loginfo(f"uav1과 드론 {drone_name}간의 충돌 위험. 거리 : {distance}")
            rospy.loginfo(f"uav1의 경로를 재계산 합니다.")
            new_target = recal_path('uav1', current_local_pose.pose.position, other_position)

            # 새로운 목표 PositionTarget 생성
            target_position = PositionTarget()
            target_position.header.stamp = rospy.Time.now()
            target_position.header.frame_id = "world"

            target_position.position.x =  new_target[0]
            target_position.position.y = new_target[1]

            local_pos_pub.publish(target_position)
        
# # 정해진 경로를 따라 이동 후 착륙
# def follow_path():
#     global path, current_local_pose, target_position, local_pos_pub, rate
#     # 실시간으로 경로를 따라 이동해야 하기 때문에 while문으로 진행
#     while not rospy.is_shutdown():
#         for point in path:
#             target_position.position.x = point[0]
#             target_position.position.y = point[1]

#             # target_position으로 이동 명령
#             local_pos_pub.publish(target_position)

#             # 드론이 목표 위치에 도달할 때까지 rate.sleep()
#             while(distance(current_local_pose, target_position) >= threshold_distance):
#                 rate.sleep()
        
#         # 목적지 도착시 착륙
#         set_mode_srv(custom_mode="AUTO.LAND")
#         break

rospy.init_node('offboard_node', anonymous=True)

# 다중드론이기 때문에 namespace를 파라미터로 받는다.
namespace = rospy.get_param("~namespace", "")

rate = rospy.Rate(20)

state_sub = rospy.Subscriber(namespace + '/mavros/state', State, state_cb)
local_pose_sub = rospy.Subscriber(namespace + '/mavros/local_position/pose', PoseStamped, local_pose_cb)
local_pos_pub = rospy.Publisher(namespace + '/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

while not rospy.is_shutdown() and current_state is None:
    rate.sleep()

target_position = PositionTarget()
target_position.position.x = current_local_pose.pose.position.x
target_position.position.y = current_local_pose.pose.position.y
target_position.position.z = current_local_pose.pose.position.z + 0.1
target_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

for i in range(100):
    local_pos_pub.publish(target_position)
    rate.sleep()

set_mode_srv = rospy.ServiceProxy(namespace + '/mavros/set_mode', SetMode)
arm_srv = rospy.ServiceProxy(namespace + '/mavros/cmd/arming', CommandBool)

# offboard 모드로 변환 후 드론을 arm 상태로 변환
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

while current_local_pose.pose.position.z < desired_altitude:
    target_position.position.z += 0.1
    local_pos_pub.publish(target_position)
    rate.sleep()

threshold_distance = 0.2 

namespace = namespace.strip('/')
rospy.loginfo(f"초기 고도 : {desired_altitude} 로 설정되었습니다.")

uav_config = drone_config.drone_configs[namespace]

start = uav_config["start"]
end = uav_config["end"]
maze = uav_config["maze"]

try : 
    path = cal_path(maze, start, end)
    if not path:
        raise ValueError("path를 찾을 수 없습니다.")
except ValueError as ve:
    print(f"오류 : {ve}")
except Exception as e:
    print(f"알고리즘 실행 도중 오류 : {e}")

if path:
    print("path 검색 SUCCESS!", path)
else:
    print("path 검색 FAIL!")

# # Douglas-Peucker 알고리즘, 경로 단순화
# epsilon = 0.5 
# simplified_path = douglas_peucker(path, epsilon)

other_drone = 'uav0'
other_pose_sub = rospy.Subscriber('/' + other_drone + '/mavros/local_position/pose', PoseStamped, position_callback, other_drone)

# target_position : Posestamped 객체
for point in path:
    target_position.position.x = point[0]  
    target_position.position.y = point[1]

    while distance2(current_local_pose, target_position) > threshold_distance:
        local_pos_pub.publish(target_position)
        rate.sleep()

# 드론 자동 착륙 모드 설정
set_mode_srv(custom_mode="AUTO.LAND")

# 노드가 종료될 때까지 대기
rospy.spin()