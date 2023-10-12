#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix

current_state = None
current_global_pose = NavSatFix()

def state_cb(msg):
    global current_state
    current_state = msg

def global_pose_cb(msg):
    global current_global_pose
    current_global_pose = msg

rospy.init_node('offboard_node', anonymous=True)
rate = rospy.Rate(20)

# Subscribe to MAVROS state and global position topics
state_sub = rospy.Subscriber('mavros/state', State, state_cb)
global_pose_sub = rospy.Subscriber('mavros/global_position/global', NavSatFix, global_pose_cb)
global_pos_pub = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)

# Wait for FCU connection
while not rospy.is_shutdown() and current_state is None:
    rate.sleep()

# latitude:위도, longitude:경도, altitude:고도
# Initialize target position (start with current position)
target_position = GeoPoseStamped()
target_position.pose.position.latitude = current_global_pose.latitude
target_position.pose.position.longitude = current_global_pose.longitude
target_position.pose.position.altitude = current_global_pose.altitude + 0.000001

# Make sure the setpoint is sent several times before switching to OFFBOARD mode
for i in range(100):
    global_pos_pub.publish(target_position)
    rate.sleep()

# Set mode to OFFBOARD and arm
set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
arm_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

response = set_mode_srv(custom_mode="OFFBOARD")
response = arm_srv(True)

# Hover for 5 seconds at the current position
hover_duration = rospy.Duration(5)
end_time = rospy.Time.now() + hover_duration
while rospy.Time.now() < end_time:
    global_pos_pub.publish(target_position)
    rate.sleep()

# NOTE: Global positions use latitude, longitude, and altitude. 
# Here, we make a very simplistic move. Real-world changes will be more nuanced than this.
# Move to a specific global position (adjustments are arbitrary and may need to be modified)
desired_altitude = 555.0
altitude_increment = 0.2

while current_global_pose.altitude < desired_altitude:
    altitude_difference = desired_altitude - current_global_pose.altitude
    altitude_increment = min(altitude_difference, 0.2)
    target_position.pose.position.altitude += altitude_increment
    global_pos_pub.publish(target_position)
    rate.sleep()

# 현재 고도에서 특정 위치로 이동
target_position.pose.position.latitude += 0.0005
target_position.pose.position.longitude += 0.0005

move_duration = rospy.Duration(10)
end_time = rospy.Time.now() + move_duration
while rospy.Time.now() < end_time:
    global_pos_pub.publish(target_position)
    rate.sleep()

# 착륙
set_mode_srv(custom_mode="AUTO.LAND")

# Spin until node is shutdown
rospy.spin()


