#!/usr/bin/env python3

import rospy
# import tkinter as tk
# from tkinter import ttk
from std_msgs.msg import Float32MultiArray

class IKTestNode:
    def __init__(self):
        rospy.init_node('ik_test_node', anonymous=True)
        
        # 토픽 발행자 생성
        self.ik_rl_pub = rospy.Publisher('/ik_rl_target', Float32MultiArray, queue_size=10)
        self.ik_ll_pub = rospy.Publisher('/ik_ll_target', Float32MultiArray, queue_size=10)
        
        # GUI 주석 처리
        # self.root = tk.Tk()
        # self.root.title("IK Test Node")
        
        # RL 프레임
        # rl_frame = ttk.LabelFrame(self.root, text="IK_RL", padding="5")
        # rl_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        # ttk.Label(rl_frame, text="X:").grid(row=0, column=0, padx=5, pady=2)
        # self.rl_x = ttk.Entry(rl_frame)
        # self.rl_x.grid(row=0, column=1, padx=5, pady=2)
        # self.rl_x.insert(0, "0")
        
        # ttk.Label(rl_frame, text="Z:").grid(row=1, column=0, padx=5, pady=2)
        # self.rl_z = ttk.Entry(rl_frame)
        # self.rl_z.grid(row=1, column=1, padx=5, pady=2)
        # self.rl_z.insert(0, "0")
        
        # ttk.Label(rl_frame, text="Th_x:").grid(row=2, column=0, padx=5, pady=2)
        # self.rl_th_x = ttk.Entry(rl_frame)
        # self.rl_th_x.grid(row=2, column=1, padx=5, pady=2)
        # self.rl_th_x.insert(0, "0")
        
        # Checkbox for using th_x_trg
        # self.use_rl_th_x = tk.BooleanVar()
        # self.use_rl_th_x.set(False)
        # ttk.Checkbutton(rl_frame, text="Use Th_x", variable=self.use_rl_th_x).grid(row=3, column=0, columnspan=2, pady=2)
        
        # ttk.Button(rl_frame, text="Send", command=self.send_rl).grid(row=4, column=0, columnspan=2, pady=5)
        
        # LL 프레임
        # ll_frame = ttk.LabelFrame(self.root, text="IK_LL", padding="5")
        # ll_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        # ttk.Label(ll_frame, text="X:").grid(row=0, column=0, padx=5, pady=2)
        # self.ll_x = ttk.Entry(ll_frame)
        # self.ll_x.grid(row=0, column=1, padx=5, pady=2)
        # self.ll_x.insert(0, "0")
        
        # ttk.Label(ll_frame, text="Z:").grid(row=1, column=0, padx=5, pady=2)
        # self.ll_z = ttk.Entry(ll_frame)
        # self.ll_z.grid(row=1, column=1, padx=5, pady=2)
        # self.ll_z.insert(0, "0")
        
        # ttk.Label(ll_frame, text="Th_x:").grid(row=2, column=0, padx=5, pady=2)
        # self.ll_th_x = ttk.Entry(ll_frame)
        # self.ll_th_x.grid(row=2, column=1, padx=5, pady=2)
        # self.ll_th_x.insert(0, "0")
        
        # Checkbox for using th_x_trg
        # self.use_ll_th_x = tk.BooleanVar()
        # self.use_ll_th_x.set(False)
        # ttk.Checkbutton(ll_frame, text="Use Th_x", variable=self.use_ll_th_x).grid(row=3, column=0, columnspan=2, pady=2)
        
        # ttk.Button(ll_frame, text="Send", command=self.send_ll).grid(row=4, column=0, columnspan=2, pady=5)
        
        # 종료 버튼
        # ttk.Button(self.root, text="Quit", command=self.quit_app).grid(row=2, column=0, pady=10)
        
        # 주기적으로 GUI 업데이트
        # self.update_gui()
        
        rospy.loginfo("IK Test Node initialized - GUI disabled")
        
        # 대신 기본 메시지를 발행
        self.send_test_messages()
        
    def send_test_messages(self):
        # 5초 대기 후 테스트 메시지 발행
        rospy.sleep(5.0)
        self.send_rl_msg(0.0, 0.0)
        rospy.sleep(2.0)
        self.send_ll_msg(0.0, 0.0)
        rospy.loginfo("Test messages sent")
        
    def send_rl_msg(self, x, z, th_x=None):
        try:
            if th_x is not None:
                msg = Float32MultiArray(data=[x, z, th_x])
                rospy.loginfo(f"Sent RL target: x={x}, z={z}, th_x={th_x}")
            else:
                msg = Float32MultiArray(data=[x, z])
                rospy.loginfo(f"Sent RL target: x={x}, z={z}")
                
            self.ik_rl_pub.publish(msg)
        except ValueError:
            rospy.logwarn("Invalid input values for RL")
            
    def send_ll_msg(self, x, z, th_x=None):
        try:
            if th_x is not None:
                msg = Float32MultiArray(data=[x, z, th_x])
                rospy.loginfo(f"Sent LL target: x={x}, z={z}, th_x={th_x}")
            else:
                msg = Float32MultiArray(data=[x, z])
                rospy.loginfo(f"Sent LL target: x={x}, z={z}")
                
            self.ik_ll_pub.publish(msg)
        except ValueError:
            rospy.logwarn("Invalid input values for LL")
            
    def send_rl(self):
        # GUI 함수 주석 처리
        pass
            
    def send_ll(self):
        # GUI 함수 주석 처리
        pass
            
    def update_gui(self):
        # GUI 함수 주석 처리
        pass
        
    def quit_app(self):
        # GUI 함수 주석 처리
        pass
        
    def run(self):
        """
        노드 실행 함수
        """
        rospy.loginfo("IK Test Node is running")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IKTestNode()
        # node.root.mainloop()  # GUI 메인루프 주석 처리
        node.run()  # 대신 ROS 노드 실행
    except rospy.ROSInterruptException:
        pass
