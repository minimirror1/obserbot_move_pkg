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

        # 테스트 함수 호출 : 초기 자세 
        self.test_origen()
        
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

    def test_origen(self):
        """
        테스트 함수: ik_rl과 ik_ll에 -200, -350 값으로 메시지를 1회 발행합니다.
        """
        rospy.loginfo("테스트 시작: ik_rl, ik_ll에 (-200, -350) 값 전송")
        
        # Float32MultiArray 메시지 생성
        rl_msg = Float32MultiArray()
        rl_msg.data = [-200.0, -350.0]
        
        ll_msg = Float32MultiArray()
        ll_msg.data = [-200.0, -350.0]
        
        # 콜백 함수를 직접 호출하여 IK 계산 및 메시지 발행
        self.rl_callback(rl_msg)
        self.ll_callback(ll_msg)
        
        rospy.loginfo("테스트 완료: 메시지 1회 발행됨")

        rospy.sleep(3)

    def test_move(self):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample.csv 파일의 좌표 값을 읽어서 5회 반복 전송
        """
        rospy.loginfo("테스트 시작: CSV 파일에서 좌표 읽어오기 (5회 반복)")
        
        # CSV 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'set_sample.csv')
        
        try:
            # CSV 파일 읽기
            with open(csv_path, 'r') as f:
                data_points = f.readlines()
            
            # 데이터 포인트 파싱
            coordinates = []
            for line in data_points:
                line = line.strip()
                if line:  # 빈 줄 무시
                    try:
                        x, z = map(float, line.split(','))
                        coordinates.append((x, z))
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(50)  # 50Hz로 실행
            total_repeats = 5  # 5회 반복
            
            # 5회 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, (x, z) in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = [x, z]
                    
                    ll_msg = Float32MultiArray()
                    ll_msg.data = [x, z]
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        rospy.loginfo(f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% (좌표: x={x:.2f}, z={z:.2f})")
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo("테스트 완료: 모든 좌표 데이터 5회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def run(self):
        rate = rospy.Rate(50)  # 10Hz
        while not rospy.is_shutdown():
            # 주기적으로 현재 저장된 각도로 메시지 발행
            self.publish_joint_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ObserbotMoveNode()
        
        # 명령줄 인수로 테스트 여부 확인
        if len(sys.argv) > 1:
            if sys.argv[1] == "move":
                # test_move 함수 실행
                node.test_move()
            elif sys.argv[1] == "origen":
                # test_origen 함수 실행 (이미 초기화 시 호출됨)
                pass
        else:
            # 기본 실행 모드
            node.run()
    except rospy.ROSInterruptException:
        pass
