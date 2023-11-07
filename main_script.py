#!/usr/bin/env python3

import rospy
import astar
import modified_astar as astar
import dp
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from dp import douglas_peucker
import test
import drone_config
from geometry_msgs.msg import Pose, Point
from modified_astar import calculate_distance as cal_dis
from modified_astar import recalculation_path as recal_path
from nav_msgs.msg import Path


current_state = None
current_local_pose = PoseStamped()
global land_publishers, path_publishers, drone_positions
land_publishers = {}
path_publishers = {} 
drone_positions = {}
CRITICAL = 5.0

def state_cb(msg):
    global current_state
    current_state = msg

def local_pose_cb(msg):
    global current_local_pose
    current_local_pose = msg
    
def distance(a, b):
    return ((a.pose.position.x - b.position.x)**2 + (a.pose.position.y - b.position.y)**2)**0.5

def get_current_pose():
    pass

def position_callback(msg, drone_name):
    # 위치 정보 저장
    drone_positions[drone_name] = msg.pose.position
    destination = drone_config.get_drone_destination(drone_name)

    # drone_positions에서 다른 드론 정보 추출
    other_drone_positions = {name: pos for name, pos in drone_positions.items() if name != drone_name}

    for other_drone_id, other_position in other_drone_positions.items():
        # drone_name과 다른 드론 간의 거리 계산
        if other_drone_id != drone_name:
            distance = cal_dis(msg.pose.position, other_position)
            if distance < CRITICAL:
                rospy.loginfo(f"드론 {drone_name}과 드론 {other_drone_id}간의 충돌 위험. 거리 : {distance}")

                # 임계값 이하일 경우 경로 재계산
                new_path = recal_path(drone_name, msg.pose.position, other_drone_positions)

                # 새 경로를 Path 메시지로 변환
                path_msg = Path()
                path_msg.header.stamp = rospy.Time.now()
                path_msg.header.frame_id = "world"

                for pose in new_path:
                    pose_stamped = PoseStamped()
                    pose_stamped.header = path_msg.head 
                    pose_stamped.pose.position.x = pose[0]
                    pose_stamped.pose.position.y = pose[1]
                    path_msg.poses.append(pose_stamped)

                # 계산된 새 경로를 발행
                path_publishers[drone_name].publish(path_msg)

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

a = astar.main(namespace)


# Douglas-Peucker 알고리즘으로 경로 단순화
epsilon = 0.5  # 높은 값은 더 단순한 경로를 생성, 낮은 값은 더 복잡한 경로를 유지
simplified_path = douglas_peucker(a, epsilon)

for point in a:
    target_position.position.x = point[0]  # x와 y 좌표를 직접 설정
    target_position.position.y = point[1]

    while distance(current_local_pose, target_position) > threshold_distance:
        local_pos_pub.publish(target_position)
        rate.sleep()

# 드론 자동 착륙 모드 설정
set_mode_srv(custom_mode="AUTO.LAND")
# 노드가 종료될 때까지 대기
rospy.spin()