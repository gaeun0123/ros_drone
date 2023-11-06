#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_msgs.msg import Empty
import modified_astar as astar
from modified_astar import main
import dp
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from dp import douglas_peucker
import test
from modified_astar import calculate_distance as cal_dis
from modified_astar import recalculation_path as recal_path
from drone_config import set_drone_destination, get_drone_destination
import drone_config

current_state = None
current_local_pose = PoseStamped()
CRITICAL = 5.0
global land_publishers, path_publishers, drone_positions
land_publishers = {}
path_publishers = {} 
drone_positions = {}

def state_cb(msg):
    global current_state
    current_state = msg
    
def distance(a, b): 
    return ((a.pose.position.x - b.position.x)**2 + (a.pose.position.y - b.position.y)**2)**0.5

def has_reached_destination(msg, destination):
    if cal_dis(msg.pose.position, destination) < 1.0:  # 1미터 이내로 가정
        return True
    
def land_callback(msg, namespace):
    land_service_topic = f"/{namespace}/mavros/cmd/land"
    rospy.wait_for_service(land_service_topic)
    try:
        land_service = rospy.ServiceProxy(land_service_topic, CommandTOL)
        # latitude, longitude, altitude는 실제 목적지 좌표에 맞게 수정해야 합니다.
        # 여기서는 일단 0으로 설정합니다.
        resp = land_service(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

# ... 통신 ... #
def position_callback(msg, drone_name):
    # 위치 정보 저장
    drone_positions[drone_name] = msg.pose.position
    destination = drone_config.get_drone_destination(drone_name)

    # drone_positions에서 다른 드론 정보 추출
    other_drone_positions = {name: pos for name, pos in drone_positions.items() if name != drone_name}

    # 목적지에 다다를 경우 착륙 명령 발행
    if has_reached_destination(msg, destination):
        land_publishers[drone_name].publish(Empty())
        rospy.loginfo(f"{drone_name} 이 목적지에 도착하여 착륙합니다.")

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
                    pose_stamped.header = path_msg.head r
                    pose_stamped.pose.position.x = pose[0]
                    pose_stamped.pose.position.y = pose[1]
                    path_msg.poses.append(pose_stamped)

                # 계산된 새 경로를 발행
                path_publishers[drone_name].publish(path_msg)

def main():
    # 노드 정의
    rospy.init_node('drone_control_node')

    # drone_config 모듈 기반 목적지 파라미터 설정
    drone_config.set_drone_destinations()

    # 네임스페이스 정의
    drone_namespaces = ['/uav0', '/uav1']

    for ns in drone_namespaces:
        # 위치 정보를 받기 위한 서브스크라이버 생성
        rospy.Subscriber(f'{ns}/position', PoseStamped, position_callback, ns)
        
        # 이륙, 착륙, 경로 설정을 위한 각각의 퍼블리셔 생성
        takeoff_publisher = rospy.Publisher(f'{ns}/takeoff', Empty, queue_size=10)
        land_publishers[ns] = rospy.Publisher(f"{ns}/land", Empty, queue_size=10)
        path_publishers[ns] = rospy.Publisher(f'{ns}/path', Path, queue_size=10)
        
        # 이륙 명령 발행
        takeoff_publisher.publish(Empty())
        rospy.sleep(5)

        # 초기 경로 계산 및 설정
        initial_path = astar.main(ns)  # A* 알고리즘으로 경로 계산

        # Path 메시지 생성
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "world"

        # 계산된 경로를 Path 메시지로 변환
        for pose_tuple in initial_path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = path_msg.header.frame_id
            pose_stamped.pose.position.x = pose_tuple[0]
            pose_stamped.pose.position.y = pose_tuple[1]
            path_msg.poses.append(pose_stamped)

        # 생성된 Path 메시지를 발행
        path_publishers[ns].publish(path_msg)


    rospy.spin()

if __name__ == '__main__':
    main()