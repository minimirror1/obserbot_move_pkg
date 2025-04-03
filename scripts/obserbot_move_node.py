#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import os

# 현재 스크립트 경로를 기준으로 상대 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
ik_dir = os.path.join(os.path.dirname(script_dir), 'obserbot_ik')
sys.path.append(ik_dir)

# 직접 obserbot_ik.py 파일 임포트
from obserbot_ik import IK_RL, IK_LL
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class ObserbotMoveNode:
    def __init__(self):
        rospy.init_node('obserbot_move_node', anonymous=True)
        
        # 토픽 구독자 생성
        self.ik_rl_sub = rospy.Subscriber('/ik_rl_target', Float32MultiArray, self.rl_callback)
        self.ik_ll_sub = rospy.Subscriber('/ik_ll_target', Float32MultiArray, self.ll_callback)
        
        # 토픽 발행자 생성
        self.joint_pub = rospy.Publisher('/canopen/multiple_joints', JointState, queue_size=10)
        
        # IK 결과 저장 변수
        self.rl_angles = [0.0, 0.0, 0.0, 0.0]  # 초기값 설정
        self.ll_angles = [0.0, 0.0, 0.0, 0.0]  # 초기값 설정
        
        rospy.loginfo("ObserbotMoveNode initialized and ready to receive IK targets")
        
    def rl_callback(self, msg):
        # msg.data = [x, z]
        x, z = msg.data
        rospy.loginfo("Received RL target: x=%.2f, z=%.2f", x, z)
        try:
            # IK 계산 수행
            th1, th2, th3, th4 = IK_RL(x, z)
            self.rl_angles = [th1, th2, th3, th4]
            rospy.loginfo("Calculated RL angles: %.2f, %.2f, %.2f, %.2f", 
                        th1, th2, th3, th4)
            
            # 조인트 상태 메시지 발행
            self.publish_joint_state()
        except Exception as e:
            rospy.logerr("Error calculating RL IK: %s", str(e))
        
    def ll_callback(self, msg):
        # msg.data = [x, z]
        x, z = msg.data
        rospy.loginfo("Received LL target: x=%.2f, z=%.2f", x, z)
        try:
            # IK 계산 수행
            th1, th2, th3, th4 = IK_LL(x, z)
            self.ll_angles = [th1, th2, th3, th4]
            rospy.loginfo("Calculated LL angles: %.2f, %.2f, %.2f, %.2f", 
                        th1, th2, th3, th4)
            
            # 조인트 상태 메시지 발행
            self.publish_joint_state()
        except Exception as e:
            rospy.logerr("Error calculating LL IK: %s", str(e))
    
    def publish_joint_state(self):
        """
        현재 계산된 IK 결과를 이용하여 JointState 메시지를 생성하고 발행합니다.
        """
        # JointState 메시지 생성
        joint_state = JointState()
        
        # 헤더 설정
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        
        # 관절 이름과 위치 설정
        joint_state.name = ['j_r1', 'j_r2', 'j_r3', 'j_r4', 'j_l1', 'j_l2', 'j_l3', 'j_l4']
        joint_state.position = self.rl_angles + self.ll_angles  # 오른쪽 다리 + 왼쪽 다리 각도
        
        # velocity와 effort는 빈 리스트로 설정 (사용하지 않음)
        joint_state.velocity = []
        joint_state.effort = []
        
        # 메시지 발행
        self.joint_pub.publish(joint_state)
        rospy.loginfo("Published joint state: %s", joint_state.position)

    def run(self):
        rate = rospy.Rate(50)  # 10Hz
        while not rospy.is_shutdown():
            # 주기적으로 현재 저장된 각도로 메시지 발행
            self.publish_joint_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ObserbotMoveNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
