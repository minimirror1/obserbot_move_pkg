import numpy as np

pi2deg = 180/np.pi

def IK_RL(x_trg, z_trg, th_x_trg = 0, th_z_trg = 0):

    # x_trg : R1 좌표계 기준 R4 좌표계의 목표 x값 (대부분 음수)
    # z_trg : R1 좌표계 기준 R4 좌표계의 목표 z값 (무조건 음수)
    # th_x_trg : 발과 지면 사이의 각도 (+면 띄어짐)
    # th_z_trg : 회전을 위한 골반 모터 각도

    # 3번째 4번째 링크 길이
    l3 = 340
    l4 = 170

    # Boundary Check

    # Calculation
    s_squ = x_trg**2 + z_trg**2
    s = np.sqrt(s_squ)

    ram1 = np.arccos(((l3**2+s_squ)-l4**2)/(2*l3*s))
    ram2 = np.arccos(((l4**2+s_squ)-l3**2)/(2*l4*s))
    gam = np.arctan(x_trg/z_trg)

    # Result
    th_r1 = th_z_trg
    th_r2 = -(ram1 + gam)
    th_r3 = -(ram1 + ram2)
    th_r4 = th_x_trg - th_r2 + th_r3

    return th_r1, th_r2, th_r3, th_r4

def IK_LL(x_trg, z_trg, th_x_trg = 0, th_z_trg = 0):

    # x_trg : R1 좌표계 기준 R4 좌표계의 목표 x값 (대부분 음수)
    # z_trg : R1 좌표계 기준 R4 좌표계의 목표 z값 (무조건 음수)
    # th_x_trg : 발과 지면 사이의 각도 (+면 띄어짐)
    # th_z_trg : 회전을 위한 골반 모터 각도

    # 3번째 4번째 링크 길이
    l3 = 340
    l4 = 170

    # Boundary Check

    # Calculation
    s_squ = x_trg**2 + z_trg**2
    s = np.sqrt(s_squ)

    ram1 = np.arccos(((l3**2+s_squ)-l4**2)/(2*l3*s))
    ram2 = np.arccos(((l4**2+s_squ)-l3**2)/(2*l4*s))
    gam = np.arctan(x_trg/z_trg)

    # Result
    th_r1 = th_z_trg
    th_r2 = ram1 + gam
    th_r3 = ram1 + ram2
    th_r4 = -th_x_trg - th_r2 + th_r3

    return th_r1, th_r2, th_r3, th_r4

# if __name__ == "__main__":

#     th1 , th2, th3, th4 = IK_RL(-50, -400)