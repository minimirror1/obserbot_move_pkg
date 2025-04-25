#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty, EmptyResponse
from dynamixel_sdk_examples.msg import SetHeadCommand, SetHeadRotation

class ObserbotMoveHeadNode:
    def __init__(self):
        rospy.init_node('obserbot_move_head_node', anonymous=True)
        
        # 토픽 발행자 생성
        self.motor_pub = rospy.Publisher('/canopen/single_motor/j_n/position', Float64, queue_size=10)
        self.head_cmd_pub = rospy.Publisher('/set_head_command', SetHeadCommand, queue_size=10)
        self.head_rotation_pub = rospy.Publisher('/set_head_rotation', SetHeadRotation, queue_size=10)
        
        # 서비스 초기화
        rospy.Service('/move_head_up', Empty, self.move_head_up)
        rospy.Service('/move_head_down', Empty, self.move_head_down)
        rospy.Service('/motion_obserbot', Empty, self.motion_obserbot)
        # 토픽 구독자 생성 (필요한 경우)
        rospy.Subscriber('/move_head', String, self.head_command_callback)
        
        rospy.loginfo("ObserbotMoveHeadNode initialized")
    
    def move_head_up(self, req=None):
        """
        머리를 올리는 함수
        1. /canopen/single_motor/j_n/position 메시지 송신
        2. 3초 대기
        3. /set_head_command 메시지 송신
        """
        rospy.loginfo("Moving head up")
        
        # 1. j_n 모터 위치 설정 메시지 송신
        motor_msg = Float64()
        motor_msg.data = 990000.0
        self.motor_pub.publish(motor_msg)
        rospy.loginfo("Published motor position: 990000.0")
        
        # 2. 3초 대기
        rospy.sleep(1.5)
        
        # 3. SetHeadCommand 메시지 송신
        head_cmd = SetHeadCommand()
        head_cmd.command = 'up'
        self.head_cmd_pub.publish(head_cmd)
        rospy.loginfo("Published head command: up")
        
        return EmptyResponse()
    
    def move_head_down(self, req=None):
        """
        머리를 내리는 함수
        j_n과 SetHeadCommand 메시지를 동시에 송신
        """
        rospy.loginfo("Moving head down")
        
        # j_n 모터 위치 설정 메시지 송신
        motor_msg = Float64()
        motor_msg.data = 5000.0
        self.motor_pub.publish(motor_msg)
        rospy.loginfo("Published motor position: 5000.0")
        
        # SetHeadCommand 메시지 송신 (동시에)
        head_cmd = SetHeadCommand()
        head_cmd.command = 'down'
        self.head_cmd_pub.publish(head_cmd)
        rospy.loginfo("Published head command: down")
        
        return EmptyResponse()
    
    def motion_obserbot(self):
        self.move_head_up()
        rospy.sleep(5.0)
 
        self.set_head_rotation(-1000)
        rospy.sleep(5.0)
        self.set_head_rotation(0)
        rospy.sleep(2.0)
        self.set_head_rotation(1000)
        rospy.sleep(5.0)
        self.set_head_rotation(0)
        rospy.sleep(5.0)
         
        self.move_head_down()
    
    def set_head_rotation(self, position):
        """
        헤드 회전 메시지를 발행하는 함수
        """
        rospy.loginfo(f"Publishing head rotation position: {position}")
        
        # SetHeadRotation 메시지 생성 및 발행
        rotation_msg = SetHeadRotation()
        rotation_msg.position = position
        self.head_rotation_pub.publish(rotation_msg)
    
    def head_command_callback(self, msg):
        """
        문자열 메시지로 머리 움직임 제어
        """
        command = msg.data.lower()
        
        if command == 'up':
            self.move_head_up()
        elif command == 'down':
            self.move_head_down()
        elif command == 'obserbot':
            self.motion_obserbot()
        else:
            rospy.logwarn("Unknown head command: %s", command)
    
    def run(self):
        """
        노드 실행 함수
        """
        rospy.loginfo("ObserbotMoveHeadNode is running")
        rospy.spin()

if __name__ == '__main__':
    try:
        # 노드 생성 및 실행
        node = ObserbotMoveHeadNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
