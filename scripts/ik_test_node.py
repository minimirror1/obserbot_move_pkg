#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Float32MultiArray

class IKTestNode:
    def __init__(self):
        rospy.init_node('ik_test_node', anonymous=True)
        
        # 토픽 발행자 생성
        self.ik_rl_pub = rospy.Publisher('/ik_rl_target', Float32MultiArray, queue_size=10)
        self.ik_ll_pub = rospy.Publisher('/ik_ll_target', Float32MultiArray, queue_size=10)
        
        # GUI 생성
        self.root = tk.Tk()
        self.root.title("IK Test Node")
        
        # RL 프레임
        rl_frame = ttk.LabelFrame(self.root, text="IK_RL", padding="5")
        rl_frame.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        ttk.Label(rl_frame, text="X:").grid(row=0, column=0, padx=5, pady=2)
        self.rl_x = ttk.Entry(rl_frame)
        self.rl_x.grid(row=0, column=1, padx=5, pady=2)
        self.rl_x.insert(0, "0")
        
        ttk.Label(rl_frame, text="Z:").grid(row=1, column=0, padx=5, pady=2)
        self.rl_z = ttk.Entry(rl_frame)
        self.rl_z.grid(row=1, column=1, padx=5, pady=2)
        self.rl_z.insert(0, "0")
        
        ttk.Button(rl_frame, text="Send", command=self.send_rl).grid(row=2, column=0, columnspan=2, pady=5)
        
        # LL 프레임
        ll_frame = ttk.LabelFrame(self.root, text="IK_LL", padding="5")
        ll_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")
        
        ttk.Label(ll_frame, text="X:").grid(row=0, column=0, padx=5, pady=2)
        self.ll_x = ttk.Entry(ll_frame)
        self.ll_x.grid(row=0, column=1, padx=5, pady=2)
        self.ll_x.insert(0, "0")
        
        ttk.Label(ll_frame, text="Z:").grid(row=1, column=0, padx=5, pady=2)
        self.ll_z = ttk.Entry(ll_frame)
        self.ll_z.grid(row=1, column=1, padx=5, pady=2)
        self.ll_z.insert(0, "0")
        
        ttk.Button(ll_frame, text="Send", command=self.send_ll).grid(row=2, column=0, columnspan=2, pady=5)
        
        # 종료 버튼
        ttk.Button(self.root, text="Quit", command=self.quit_app).grid(row=2, column=0, pady=10)
        
        # 주기적으로 GUI 업데이트
        self.update_gui()
        
    def send_rl(self):
        try:
            x = float(self.rl_x.get())
            z = float(self.rl_z.get())
            msg = Float32MultiArray(data=[x, z])
            self.ik_rl_pub.publish(msg)
            rospy.loginfo(f"Sent RL target: x={x}, z={z}")
        except ValueError:
            rospy.logwarn("Invalid input values for RL")
            
    def send_ll(self):
        try:
            x = float(self.ll_x.get())
            z = float(self.ll_z.get())
            msg = Float32MultiArray(data=[x, z])
            self.ik_ll_pub.publish(msg)
            rospy.loginfo(f"Sent LL target: x={x}, z={z}")
        except ValueError:
            rospy.logwarn("Invalid input values for LL")
            
    def update_gui(self):
        self.root.update()
        rospy.sleep(0.1)
        self.update_gui()
        
    def quit_app(self):
        self.root.quit()
        rospy.signal_shutdown("User requested shutdown")

if __name__ == '__main__':
    try:
        node = IKTestNode()
        node.root.mainloop()
    except rospy.ROSInterruptException:
        pass
