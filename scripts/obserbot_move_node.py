#!/usr/bin/env python3

import rospy
import numpy as np
import sys
import os
import math

# 현재 스크립트 경로를 기준으로 상대 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
ik_dir = os.path.join(os.path.dirname(script_dir), 'obserbot_ik')
sys.path.append(ik_dir)

# 직접 obserbot_ik.py 파일 임포트
from obserbot_ik import IK_RL, IK_LL
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header

def euler_from_quaternion(quaternion):
    """
    쿼터니언을 오일러 각(롤, 피치, 요)으로 변환하는 함수
    
    Args:
        quaternion: [x, y, z, w] 형식의 쿼터니언
    
    Returns:
        tuple: (롤, 피치, 요) 각도(라디안)
    """
    x, y, z, w = quaternion
    
    # 롤 (x축 회전)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # 피치 (y축 회전)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 90도 if out of range
    else:
        pitch = math.asin(sinp)
        
    # 요 (z축 회전)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)

class ObserbotMoveNode:
    def __init__(self):
        rospy.init_node('obserbot_move_node', anonymous=True)
        
        # 토픽 구독자 생성
        self.ik_rl_sub = rospy.Subscriber('/ik_rl_target', Float32MultiArray, self.rl_callback)
        self.ik_ll_sub = rospy.Subscriber('/ik_ll_target', Float32MultiArray, self.ll_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # 토픽 발행자 생성
        self.joint_pub = rospy.Publisher('/canopen/multiple_joints', JointState, queue_size=10)
        
        # IK 결과 저장 변수
        self.rl_angles = [0.0, 0.0, 0.0, 0.0]  # 초기값 설정
        self.ll_angles = [0.0, 0.0, 0.0, 0.0]  # 초기값 설정
        
        # IMU 데이터 저장 변수
        self.imu_data = None
        
        # IMU 자세각 저장 변수
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        rospy.loginfo("ObserbotMoveNode initialized and ready to receive IK targets")

        
        

    def rl_callback(self, msg):
        # msg.data = [x, z] 또는 [x, z, th_x_trg]
        if len(msg.data) >= 2:
            x, z = msg.data[0], msg.data[1]
            th_x_trg = msg.data[2] if len(msg.data) > 2 else None
            
            rospy.loginfo("Received RL target: x=%.2f, z=%.2f%s", 
                        x, z, f", th_x_trg={th_x_trg}" if th_x_trg is not None else "")
            try:
                # IK 계산 수행
                if th_x_trg is not None:
                    # th_x_trg 매개변수가 있는 경우의 처리
                    th1, th2, th3, th4 = IK_RL(x, z, th_x_trg)
                else:
                    # 기존 방식대로 처리
                    th1, th2, th3, th4 = IK_RL(x, z)
                
                self.rl_angles = [th1, th2, th3, th4]
                rospy.loginfo("Calculated RL angles: %.2f, %.2f, %.2f, %.2f", 
                            th1, th2, th3, th4)
                
                # 조인트 상태 메시지 발행
                self.publish_joint_state()
            except Exception as e:
                rospy.logerr("Error calculating RL IK: %s", str(e))
        else:
            rospy.logerr("Received invalid RL target data format. Expected at least [x, z]")
        
    def ll_callback(self, msg):
        # msg.data = [x, z] 또는 [x, z, th_x_trg]
        if len(msg.data) >= 2:
            x, z = msg.data[0], msg.data[1]
            th_x_trg = msg.data[2] if len(msg.data) > 2 else None
            
            rospy.loginfo("Received LL target: x=%.2f, z=%.2f%s", 
                        x, z, f", th_x_trg={th_x_trg}" if th_x_trg is not None else "")
            try:
                # IK 계산 수행
                if th_x_trg is not None:
                    # th_x_trg 매개변수가 있는 경우의 처리
                    th1, th2, th3, th4 = IK_LL(x, z, th_x_trg)
                else:
                    # 기존 방식대로 처리
                    th1, th2, th3, th4 = IK_LL(x, z)
                
                self.ll_angles = [th1, th2, th3, th4]
                rospy.loginfo("Calculated LL angles: %.2f, %.2f, %.2f, %.2f", 
                            th1, th2, th3, th4)
                
                # 조인트 상태 메시지 발행
                self.publish_joint_state()
            except Exception as e:
                rospy.logerr("Error calculating LL IK: %s", str(e))
        else:
            rospy.logerr("Received invalid LL target data format. Expected at least [x, z]")
       
    
    def imu_callback(self, msg):
        """
        IMU 데이터를 받아 처리하는 콜백 함수
        """
        self.imu_data = msg
 
        # 회전 데이터 추출
        quat = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # 각도를 라디안에서 도(degree)로 변환
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)
        
        rospy.loginfo("IMU 각도: 롤=%.2f°, 피치=%.2f°, 요=%.2f°", roll_deg, pitch_deg, yaw_deg)
        
        # 가속도 데이터도 필요하다면 아래 주석을 해제
        # rospy.loginfo("선형 가속도: x=%.2f, y=%.2f, z=%.2f",
        #               msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
    
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

    def test_origen(self, x_R=-200.0, z_R=-375.0, x_L=-100.0, z_L=-375.0):
        """
        테스트 함수: ik_rl과 ik_ll에 초기 위치값을 설정합니다.
        
        Args:
            x_R (float): 오른쪽 다리 x 좌표
            z_R (float): 오른쪽 다리 z 좌표
            x_L (float): 왼쪽 다리 x 좌표
            z_L (float): 왼쪽 다리 z 좌표
        """
        rospy.loginfo("테스트 시작: ik_rl, ik_ll에 초기 위치 값 전송")
        
        # Float32MultiArray 메시지 생성
        rl_msg = Float32MultiArray()
        rl_msg.data = [x_R, z_R]
        
        ll_msg = Float32MultiArray()
        ll_msg.data = [x_L, z_L]

        # 콜백 함수를 직접 호출하여 IK 계산 및 메시지 발행
        self.rl_callback(rl_msg)
        self.ll_callback(ll_msg)
        
        rospy.loginfo("테스트 완료: 초기 위치 메시지 발행됨")

        rospy.sleep(3)

    def test_move(self, total_repeats=1):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample.csv 파일의 좌표 값을 읽어서 지정된 횟수만큼 반복 전송
        
        Args:
            total_repeats (int): 데이터 반복 전송 횟수 (기본값: 1)
        """
        rospy.loginfo(f"테스트 시작: CSV 파일에서 좌표 읽어오기 ({total_repeats}회 반복)")
        
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
                        values = list(map(float, line.split(',')))
                        # 2개 또는 3개의 값을 지원 (x, z) 또는 (x, z, th_x_trg)
                        if len(values) >= 2:
                            coordinates.append(values)
                        else:
                            rospy.logwarn(f"데이터 형식 오류: {line} - 최소 x, z 두 개의 값이 필요합니다.")
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(50)  # 50Hz로 실행
            
            # 지정된 횟수만큼 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, values in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = values
                    
                    ll_msg = Float32MultiArray()
                    ll_msg.data = values
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        log_msg = f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% (좌표: x={values[0]:.2f}, z={values[1]:.2f}"
                        if len(values) > 2:
                            log_msg += f", th_x_trg={values[2]:.2f}"
                        log_msg += ")"
                        rospy.loginfo(log_msg)
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo(f"테스트 완료: 모든 좌표 데이터 {total_repeats}회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def test_move_2(self, total_repeats=1):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample_2.csv 파일의 좌표 값을 읽어서 지정된 횟수만큼 반복 전송
        CSV 형식: x_R, z_R, th_R, x_L, z_L, th_L 순서
        
        Args:
            total_repeats (int): 데이터 반복 전송 횟수 (기본값: 1)
        """
        rospy.loginfo(f"테스트 시작: CSV 파일에서 다리 좌표 데이터 읽어오기 ({total_repeats}회 반복)")
        
        # CSV 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'set_sample_2.csv')

        self.test_origen(-200,-375,-100,-375)
        
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
                        values = list(map(float, line.split(',')))
                        # x_R, z_R, th_R, x_L, z_L, th_L 형식을 처리
                        if len(values) >= 6:
                            x_R, z_R, th_R, x_L, z_L, th_L = values[:6]
                            coordinates.append((x_R, z_R, th_R, x_L, z_L, th_L))
                        else:
                            rospy.logwarn(f"데이터 형식 오류: {line} - 6개의 값이 필요합니다 (x_R, z_R, th_R, x_L, z_L, th_L)")
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(50)  # 50Hz로 실행
            
            # 지정된 횟수만큼 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, (x_R, z_R, th_R, x_L, z_L, th_L) in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # RL 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = [x_R, z_R, th_R]
                    
                    # LL 메시지 생성 및 발행
                    ll_msg = Float32MultiArray()
                    ll_msg.data = [x_L, z_L, th_L]
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        rospy.loginfo(f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% "
                                      f"(RL: x={x_R:.2f}, z={z_R:.2f}, th={th_R:.2f}, "
                                      f"LL: x={x_L:.2f}, z={z_L:.2f}, th={th_L:.2f})")
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo(f"테스트 완료: 모든 좌표 데이터 {total_repeats}회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def test_move_3(self, total_repeats=1):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample_3.csv 파일의 좌표 값을 읽어서 지정된 횟수만큼 반복 전송
        CSV 형식: x_R, z_R, th_R, x_L, z_L, th_L 순서
        
        Args:
            total_repeats (int): 데이터 반복 전송 횟수 (기본값: 1)
        """
        rospy.loginfo(f"테스트 시작: CSV 파일에서 다리 좌표 데이터 읽어오기 ({total_repeats}회 반복)")
        
        # CSV 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'set_sample_3.csv')

        # 테스트 함수 호출 : 초기 자세 (기본값 사용)
        self.test_origen(-300,-375,-200,-375)
        
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
                        values = list(map(float, line.split(',')))
                        # x_R, z_R, th_R, x_L, z_L, th_L 형식을 처리
                        if len(values) >= 6:
                            x_R, z_R, th_R, x_L, z_L, th_L = values[:6]
                            coordinates.append((x_R, z_R, th_R, x_L, z_L, th_L))
                        else:
                            rospy.logwarn(f"데이터 형식 오류: {line} - 6개의 값이 필요합니다 (x_R, z_R, th_R, x_L, z_L, th_L)")
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(100)  # 50Hz로 실행
            
            # 지정된 횟수만큼 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, (x_R, z_R, th_R, x_L, z_L, th_L) in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # RL 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = [x_R, z_R, th_R]
                    
                    # LL 메시지 생성 및 발행
                    ll_msg = Float32MultiArray()
                    ll_msg.data = [x_L, z_L, th_L]
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        rospy.loginfo(f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% "
                                      f"(RL: x={x_R:.2f}, z={z_R:.2f}, th={th_R:.2f}, "
                                      f"LL: x={x_L:.2f}, z={z_L:.2f}, th={th_L:.2f})")
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo(f"테스트 완료: 모든 좌표 데이터 {total_repeats}회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def test_move_4(self, total_repeats=1):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample_4.csv 파일의 좌표 값을 읽어서 지정된 횟수만큼 반복 전송
        CSV 형식: x_R, z_R, th_R, x_L, z_L, th_L 순서
        
        Args:
            total_repeats (int): 데이터 반복 전송 횟수 (기본값: 1)
        """
        rospy.loginfo(f"테스트 시작: CSV 파일에서 다리 좌표 데이터 읽어오기 ({total_repeats}회 반복)")
        
        # CSV 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'set_sample_4.csv')

        # 테스트 함수 호출 : 초기 자세 (기본값 사용)
        self.test_origen(-250,-360,-150,-360)        
        
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
                        values = list(map(float, line.split(',')))
                        # x_R, z_R, th_R, x_L, z_L, th_L 형식을 처리
                        if len(values) >= 6:
                            x_R, z_R, th_R, x_L, z_L, th_L = values[:6]
                            coordinates.append((x_R, z_R, th_R, x_L, z_L, th_L))
                        else:
                            rospy.logwarn(f"데이터 형식 오류: {line} - 6개의 값이 필요합니다 (x_R, z_R, th_R, x_L, z_L, th_L)")
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(100)  # 50Hz로 실행
            
            # 지정된 횟수만큼 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, (x_R, z_R, th_R, x_L, z_L, th_L) in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # RL 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = [x_R, z_R, th_R]
                    
                    # LL 메시지 생성 및 발행
                    ll_msg = Float32MultiArray()
                    ll_msg.data = [x_L, z_L, th_L]
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        rospy.loginfo(f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% "
                                      f"(RL: x={x_R:.2f}, z={z_R:.2f}, th={th_R:.2f}, "
                                      f"LL: x={x_L:.2f}, z={z_L:.2f}, th={th_L:.2f})")
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo(f"테스트 완료: 모든 좌표 데이터 {total_repeats}회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def test_move_5(self, total_repeats=1):
        """
        데이터 파일에서 값을 읽어 순차적으로 송신하는 테스트 함수
        set_sample_4.csv 파일의 좌표 값을 읽어서 지정된 횟수만큼 반복 전송
        CSV 형식: x_R, z_R, th_R, x_L, z_L, th_L 순서
        
        Args:
            total_repeats (int): 데이터 반복 전송 횟수 (기본값: 1)
        """
        rospy.loginfo(f"테스트 시작: CSV 파일에서 다리 좌표 데이터 읽어오기 ({total_repeats}회 반복)")
        
        # CSV 파일 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, 'set_sample_5.csv')

        # 테스트 함수 호출 : 초기 자세 (기본값 사용)
        self.test_origen(-250,-360,-170,-360)  
         
            
        
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
                        values = list(map(float, line.split(',')))
                        # x_R, z_R, th_R, x_L, z_L, th_L 형식을 처리
                        if len(values) >= 6:
                            x_R, z_R, th_R, x_L, z_L, th_L = values[:6]
                            coordinates.append((x_R, z_R, th_R, x_L, z_L, th_L))
                        else:
                            rospy.logwarn(f"데이터 형식 오류: {line} - 6개의 값이 필요합니다 (x_R, z_R, th_R, x_L, z_L, th_L)")
                    except ValueError:
                        rospy.logwarn(f"잘못된 데이터 형식 무시: {line}")
            
            rospy.loginfo(f"총 {len(coordinates)}개의 좌표점 로드 완료")
            
            # 데이터 포인트가 없으면 종료
            if not coordinates:
                rospy.logerr("유효한 좌표 데이터가 없습니다.")
                return
            
            rate = rospy.Rate(80)  # 50Hz로 실행
            
            # 지정된 횟수만큼 반복 전송
            for repeat in range(1, total_repeats + 1):
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 시작")
                
                # 각 좌표점에 대해 메시지 발행
                for i, (x_R, z_R, th_R, x_L, z_L, th_L) in enumerate(coordinates):
                    if rospy.is_shutdown():
                        return
                    
                    # RL 메시지 생성 및 발행
                    rl_msg = Float32MultiArray()
                    rl_msg.data = [x_R, z_R, th_R]
                    
                    # LL 메시지 생성 및 발행
                    ll_msg = Float32MultiArray()
                    ll_msg.data = [x_L, z_L, th_L]
                    
                    # 콜백 함수 호출
                    self.rl_callback(rl_msg)
                    self.ll_callback(ll_msg)
                    
                    # 현재 진행 상황 로깅 (10% 간격으로)
                    if i % (len(coordinates) // 10) == 0 or i == len(coordinates) - 1:
                        progress = (i / (len(coordinates) - 1)) * 100
                        rospy.loginfo(f"반복 {repeat}/{total_repeats} - 진행률: {progress:.1f}% "
                                      f"(RL: x={x_R:.2f}, z={z_R:.2f}, th={th_R:.2f}, "
                                      f"LL: x={x_L:.2f}, z={z_L:.2f}, th={th_L:.2f})")
                    
                    rate.sleep()
                
                rospy.loginfo(f"반복 {repeat}/{total_repeats} 완료")
            
            rospy.loginfo(f"테스트 완료: 모든 좌표 데이터 {total_repeats}회 반복 전송 완료")
            
        except Exception as e:
            rospy.logerr(f"데이터 처리 중 오류 발생: {str(e)}")

    def move_origen(self):
        self.test_origen(-300,-375,-200,-375)
        rospy.sleep(3)

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
            # 두 번째 인수가 있으면 total_repeats 값으로 사용
            total_repeats = 1  # 기본값 설정
            if len(sys.argv) > 2:
                try:
                    total_repeats = int(sys.argv[2])
                except ValueError:
                    rospy.logwarn(f"Invalid total_repeats value: {sys.argv[2]}. Using default: 1")
            
            if sys.argv[1] == "move":
                # test_move 함수 실행
                node.test_move(total_repeats)    
            elif sys.argv[1] == "move_2":
                node.test_move_2(total_repeats)
            elif sys.argv[1] == "move_3":
                node.test_move_3(total_repeats)
            elif sys.argv[1] == "move_4":
                node.test_move_4(total_repeats)
            elif sys.argv[1] == "move_5":
                node.test_move_5(total_repeats)
            elif sys.argv[1] == "origen":
                node.move_origen()
                # test_origen 함수 실행 (이미 초기화 시 호출됨)
                pass
        else:
            # 기본 실행 모드
            node.run()
    except rospy.ROSInterruptException:
        pass
