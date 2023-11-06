#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Pose, Point, Twist
from std_msgs.msg import Empty
import modified_astar
from drone_config import drone_configs as config
from mavros_msgs.srv import CommandBool, SetMode

namespace = rospy.get_namespace().strip('/')
other_drone_positions = {} # 다른 드론의 위치
current_local_pose = None  # 현재 드론의 위치
path = []  # 현재 계산된 경로
threshold_distance = 1.0  # 목적지 도착 판단 임계값
CRITICAL_DISTANCE = 5.0  # 다른 드론과의 안전한 거리 임계값
other_drones = ['drone1', 'drone2']

# 퍼블리셔, 서브스크라이버 정의
local_pos_pub = rospy.Publisher(f"{namespace}/setpoint_position/local", Pose, queue_size=10)
takeoff_pub = rospy.Publisher(f"{namespace}/takeoff", Empty, queue_size=10)
land_pub = rospy.Publisher(f"{namespace}/land", Empty, queue_size=10)

arming_client = rospy.ServiceProxy(f"{namespace}/mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy(f"{namespace}/mavros/set_mode", SetMode)

def pose_callback(msg):
    global current_local_pose
    current_local_pose = msg

def other_drone_pose_callback(msg):
    global other_drone_positions
    other_drone_positions['uav0'] = msg

# 드론 시동
def set_arm_state(arm, timeout):
    rospy.loginfo(f"{'Arming' if arm else 'Disarming'} the drone")
    for _ in range(timeout):
        if arming_client(arm).success:
            rospy.loginfo("드론 시동 준비 완료")
            return True
        rospy.sleep(1)
    return False

# 모드 변환
def set_flight_mode(mode, timeout):
    rospy.loginfo(f"Setting flight mode to {mode}")
    for _ in range(timeout):
        try:
            response = set_mode_client(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo("모드 변환 되었습니다.")
                return True
            else:
                rospy.logwarn("모드 전환 요청은 보냈으나, 전환에 실패하였습니다.")
        except rospy.ServiceException as e:
            rospy.logerr("서비스 호출 실패: %s" % e)
        rospy.sleep(1)
    return False

# 이륙
def takeoff(target_altitude):
    rospy.loginfo("Preparing to take off. .")
    pose = Pose()
    pose.position.z = target_altitude

    for _ in range(10):
        local_pos_pub.publish(pose)
        rospy.sleep(0.1)

    rospy.loginfo("Taking off.")
    takeoff_pub.publish(Empty())

    # 고도에 도달하기 전까지 대기
    while current_local_pose.position.z < target_altitude - 0.5:  
        rospy.sleep(0.1)

# 착륙
def land():
    rospy.loginfo("Landing.")
    land_pub.publish(Empty())

# 초기 경로 계산
def calculate_path():
    global path
    uav_config = config.get(namespace)
    if not uav_config:
        rospy.logerr("다음과 같은 namespace의 드론이 없습니다. : %s", namespace)
        return
    
    start = uav_config["start"]
    end = uav_config["end"]
    maze = uav_config["maze"]
    
    # astar 경로 계산
    path = modified_astar.astar(maze, start, end)

# 경로 재계산
def recalculate_path_if_necessary():
    global path

    for _, other_pose in other_drone_positions.items():
        # 드론 간의 거리가 임계값 이하일 경우 경로 재계산
        if modified_astar.calculate_distance(current_local_pose.position, other_pose) < CRITICAL_DISTANCE:
            rospy.loginfo("충돌 위험. 경로를 재계산 합니다.")
            path = modified_astar.recalculation_path(namespace, current_local_pose, other_drone_positions)
            break

# 경로 따라 이동
def follow_path():
    for point in path:
        target_pose = Pose()
        target_pose.position = Point(point[0], point[1], 0)
        local_pos_pub.publish(target_pose)

        rospy.sleep(2)

# 드론 상태 확인
def check_flight_readiness(timeout):
    rospy.loginfo("비행 준비 상태를 확인합니다.")
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < timeout:
        # 여기서는 예시로 current_local_pose의 존재 여부만 확인합니다.
        # 실제로는 GPS 상태, 배터리 상태, 센서 상태 등을 확인할 필요가 있습니다.
        if current_local_pose is not None:
            rospy.loginfo("드론이 비행 준비 상태입니다.")
            return True
        rospy.sleep(1)
    rospy.loginfo("드론이 비행 준비 상태가 아닙니다.")
    return False

def main():
    # 서브스크라이버 정의
    rospy.Subscriber(f"{namespace}/local_position/pose", Pose, pose_callback)
    rospy.Subscriber("/uav0/local_position/pose", Pose, other_drone_pose_callback)

    '''
    main
    '''
    # 초기 경로 계산
    calculate_path()

    # 드론 상태 실시간 체크
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        if modified_astar.calculate_distance(current_local_pose.position, Pose(Point(*config.get(namespace)["end"]), None).position) <= threshold_distance:
            rospy.loginfo("목적지 도착, 착륙합니다.")
            land()
            break
        
        # 드론 간의 거리를 실시간으로 확인, 필요한 경우 경로를 재계산
        recalculate_path_if_necessary()

        # 경로를 따라 드론이 이동
        follow_path()

        rate.sleep()

if __name__ == '__main__':
    try:
        # ROS node 초기화
        rospy.init_node('drone_control_node', anonymous=True)

        # 비행 준비 상태 확인
        if not check_flight_readiness(10):
            rospy.loginfo("비행 준비 상태 확인 실패")
            sys.exit(0)  # 비행 준비 상태가 아니면 프로그램 종료

        # 모드 OFFBOARD로 전환
        if set_flight_mode("OFFBOARD", 5):
            # 시동 걸기
            if set_arm_state(True, 5):
                # 이륙 명령
                takeoff(10.0)
                # 메인 로직
                main()
            else:
                rospy.loginfo("모드 전환 Failed")
        else:
            rospy.loginfo("시동 Failed")

    except rospy.ROSInterruptException:
        pass
