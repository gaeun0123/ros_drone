import rospy
import tf
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# ROS 노드 초기화
rospy.init_node('multi_drone_navigation')

# 드론 A와 드론 B의 move_base 클라이언트 설정
client_a = actionlib.SimpleActionClient('/uav0/move_base', MoveBaseAction)
client_b = actionlib.SimpleActionClient('/uav1/move_base', MoveBaseAction)
client_a.wait_for_server()
client_b.wait_for_server()

# tf 리스너 설정
listener = tf.TransformListener()

def set_goal(client, drone_from, drone_to):
    try:
        # 드론 from의 로컬 좌표계에서 드론 to의 최신 위치를 기다립니다.
        listener.waitForTransform(drone_from, drone_to, rospy.Time(0), rospy.Duration(4.0))

        # 드론 to의 위치를 드론 from의 로컬 좌표계로 변환합니다.
        (trans, rot) = listener.lookupTransform(drone_from, drone_to, rospy.Time(0))

        # 변환된 위치를 사용하여 드론 from의 목적지를 설정합니다.
        goal = PoseStamped()
        goal.header.frame_id = drone_from
        goal.pose.position.x = trans[0]
        goal.pose.position.y = trans[1]
        goal.pose.orientation.w = 1.0  # 간단한 예시를 위해 z 축을 중심으로 한 회전만 고려합니다.

        # move_base goal을 설정하고 명령을 보냅니다.
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = goal
        client.send_goal(move_base_goal)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF 변환에 실패: %s" % e)

# 드론 A와 B의 위치를 서로의 목적지로 설정하는 루프
while not rospy.is_shutdown():
    set_goal(client_a, '/drone_a/base_link', '/drone_b/base_link')
    set_goal(client_b, '/drone_b/base_link', '/drone_a/base_link')
    rospy.sleep(5)  # 5초 간격으로 위치 업데이트
