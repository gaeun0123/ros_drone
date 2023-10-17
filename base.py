#!/usr/bin/env python3

# qgc에서 waypoint 지정 후 waypoint를 받아 경로에 Astar 알고리즘 입히는 코드
# global position 기반

import rospy
from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from astar import main as astar_main
import dp
from dp import douglas_peucker
from drone_config import drone_configs
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointSetCurrent


class MultiDroneNode:
    def __init__(self, drone_namespace):
        self.namespace = drone_namespace

        # 상태 및 위치 정보 초기화
        self.current_state = State()
        self.current_global_pose = NavSatFix()

        # waypoint를 저장하기 위한 리스트
        self.waypoints = []
        self.path = []

        # Subscriber 및 Publisher 초기화
        self.state_sub = rospy.Subscriber(self.namespace + '/mavros/state', State, self.state_cb)
        self.global_pose_sub = rospy.Subscriber(self.namespace + '/mavros/global_position/global', NavSatFix, self.global_pose_cb)
        self.global_pose_pub = rospy.Publisher(self.namespace + '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)
        
        self.waypoints_sub = rospy.Subscriber(self.namespace + '/mavros/mission/waypoints', WaypointList, self.waypoint_cb)
        self.arming_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
        self.land_client = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)

    def state_cb(self, msg):
        rospy.loginfo("Received state message")
        self.current_state = msg

    def global_pose_cb(self, msg):
        self.current_global_pose = msg

    # 웨이포인트 정보 받기
    def waypoint_cb(self, data):
        self.waypoints = data.waypoints

    def takeoff(self, altitude=20.0):
        print("Namespace : " + self.namespace)
        if not self.current_state:
            rospy.logwarn("Current state not yet received!")
            return
        
        if self.current_state.mode != "GUIDED":
            self.set_mode("GUIDED")
        print("guided mode로 변경되었습니다.")

        self.arm()
        takeoff_cmd = CommandTOL()
        takeoff_cmd.altitude = altitude
        self.takeoff_client(min_pitch=0, yaw=0, latitude=0, longitude=0, altitude=altitude)


    def land(self):
        self.set_mode("LAND")

    def set_mode(self, mode):
        rospy.wait_for_service(self.namespace + '/mavros/set_mode')
        try:
            self.set_mode_client(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def arm(self):
        rospy.wait_for_service(self.namespace + '/mavros/cmd/arming')
        try:
            self.arming_client(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def plan_path_with_astar(self):
        start_global = (self.waypoints[0].x_lat, self.waypoints[0].y_long)
        end_global = (self.waypoints[-1].x_lat, self.waypoints[-1].y_long)

        print("way포인트의 글로벌 값")
        print(start_global)
        print(end_global)
        
        # 웨이포인트의 첫 지점과 마지막 지점을 출발지, 목적지로 설정 후 astar 알고리즘 경로 찾기
        self.path = astar_main(self.namespace, start_global, end_global)

    # waypoints에 따른 미션 서비스 클라이언트 생성
    def send_mavros_mission(self, waypoints):
        try:
            service = rospy.ServiceProxy(self.namespace + '/mavros/mission/push', WaypointPush)
            response = service(0, waypoints) # 0 is the starting waypoint
            if not response.success:
                rospy.logerr("Failed to send waypoints to MAVROS")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def fly_along_path(self):
        mavros_waypoints = self.convert_path_to_mavros_waypoints(self.path)

        # 변환된 웨이포인트를 MAVROS에 전송합니다.
        if not self.send_mavros_mission(mavros_waypoints):
            rospy.logerr("Failed to send waypoints to MAVROS.")
            return

        # AUTO.MISSION 모드로 변경합니다.
        if not self.set_mission_mode():
            rospy.logerr("Failed to set AUTO.MISSION mode.")
            return

        # 첫 번째 웨이포인트를 시작점으로 설정하고 미션을 시작합니다.
        if not self.start_mission():
            rospy.logerr("Failed to start the mission from the first waypoint.")
            return

        rospy.loginfo("Mission started. Drone is flying along the path.")


    
    def start_mission(self):
        try :
            # 시작 웨이포인트를 설정합니다. 여기서는 첫 웨이포인트를 시작점으로 합니다.
            set_waypoint_service = rospy.ServiceProxy(self.namespace + '/mavros/mission/set_current', WaypointSetCurrent)
            response = set_waypoint_service(0)  # 첫 번째 웨이포인트를 시작점으로 설정
            if not response.success:
                rospy.logerr("Failed to set start waypoint")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    # mission모드로 변환  
    def set_mission_mode(self):
        try:
            set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
            response = set_mode_service(0, "AUTO.MISSION")
            if not response.mode_sent: # `mode_sent`는 `SetMode` 서비스의 응답 필드 중 하나입니다.
                rospy.logerr("Failed to set AUTO.MISSION mode")
                return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False
    
    # 경로를 waypoints로 변환
    def convert_path_to_mavros_waypoints(self, path, altitude=20.0):
        mavros_waypoints = []

        for position in path:
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = 0  # 홀드 시간
            wp.param2 = 0  # 허용 편차
            wp.param3 = 0  # 패스 스토리지 (여기서는 사용하지 않음)
            wp.param4 = 0  # 요구 각도 (여기서는 사용하지 않음)
            wp.x_lat = position[0]
            wp.y_long = position[1]
            wp.z_alt = altitude
            mavros_waypoints.append(wp)

        return mavros_waypoints
    
# 실행 파일로 실행될 경우 name은 main으로 설정
# import 파일로 사용되어질 경우에는 name이 해당 스크립트 파일 명으로 변경
if __name__ == '__main__':
    # ROS 노드 초기화
    # multi_drone_node라는 노드 생성
    rospy.init_node('multi_drone_node', anonymous=True)

    # namespace를 파라미터로 받기
    drone_namespaces = rospy.get_param("~namespaces", "").split(',')
    print("Namespaces value from parameter server:", rospy.get_param("~namespaces", "Default value"))

    print("드론 객체 생성 완료")
    print(drone_namespaces)

     # 각 드론의 노드 객체를 저장할 리스트 초기화
    drone_nodes = []

    # 각 드론에 대해 노드 객체 생성
    for ns in drone_namespaces:
        drone_nodes.append(MultiDroneNode(ns.strip()))

    for drone_node in drone_nodes :
        drone_node.takeoff()
        rospy.sleep(5)  # takeoff 후에 약간의 대기 시간
        drone_node.plan_path_with_astar()
        drone_node.fly_along_path()
        print("비행완료")
    
    # ROS 노드를 실행 상태로 유지하며 메시지 계속 수신하도록 함
    rospy.spin()