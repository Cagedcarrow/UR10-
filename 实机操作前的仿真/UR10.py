import math
import numpy as np

class UR10_1:
    def jointAnglToPipeEnd(theta1, theta2, theta3, theta4, theta5, theta6):

        # 调用前向运动学函数计算机械臂末端执行器的位置和姿态
        pos, T = UR10_1.forwardKinematicsUR10(theta1, theta2, theta3, theta4, theta5, theta6)

        # 调用flangeToPipeEnd函数将末端执行器的位置和姿态转换为喷管末端的位置和姿态
        pipeEndP, pipeEndT = UR10_1.flangeToPipeEnd(pos, T)

        return pipeEndP, pipeEndT

    def flangeToPipeEnd(flangeP, flangeT):

        # 扁口喷头中心到连接件法兰底面中心的向量，在喷头坐标系下
        pPipeEndInFlange = np.array([-0.146, -0.272, 0.00739])

        # 喷管末端相对于法兰的变换矩阵（根据具体朝向关系选择）
        pipeEndRelateToFlangeT = np.array([[1, 0, 0],
                                           [0, 0, -1],
                                           [0, 1, 0]])

        # 计算喷管末端的姿态
        pipeEndT = flangeT @ pipeEndRelateToFlangeT

        # 计算喷管末端的位置
        pipeEndP = flangeP + flangeT @ pPipeEndInFlange

        return pipeEndP, pipeEndT

    def forwardKinematicsUR10(theta1, theta2, theta3, theta4, theta5, theta6):

        # UR10的DH参数
        d1, a1, d4, a4 = 0.128, 0, 0.1639, 0
        d2, a2, d5, a5 = 0, -0.6129, 0.1157, 0
        d3, a3, d6, a6 = 0, -0.5716, 0.0922, 0

        # 计算三角函数值
        s1, c1 = np.sin(theta1), np.cos(theta1)
        s2, c2 = np.sin(theta2), np.cos(theta2)
        s3, c3 = np.sin(theta3), np.cos(theta3)
        s4, c4 = np.sin(theta4), np.cos(theta4)
        s5, c5 = np.sin(theta5), np.cos(theta5)
        s6, c6 = np.sin(theta6), np.cos(theta6)

        # 定义变换矩阵
        T01 = np.array([[c1, 0, s1, 0],
                        [s1, 0, -c1, 0],
                        [0, 1, 0, d1],
                        [0, 0, 0, 1]])
        T12 = np.array([[c2, -s2, 0, a2 * c2],
                        [s2, c2, 0, a2 * s2],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        T23 = np.array([[c3, -s3, 0, a3 * c3],
                        [s3, c3, 0, a3 * s3],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        T34 = np.array([[c4, 0, s4, 0],
                        [s4, 0, -c4, 0],
                        [0, 1, 0, d4],
                        [0, 0, 0, 1]])
        T45 = np.array([[c5, 0, -s5, 0],
                        [s5, 0, c5, 0],
                        [0, -1, 0, d5],
                        [0, 0, 0, 1]])
        T56 = np.array([[c6, -s6, 0, 0],
                        [s6, c6, 0, 0],
                        [0, 0, 1, d6],
                        [0, 0, 0, 1]])

        # 计算总的变换矩阵
        THomogeneous = T01 @ T12 @ T23 @ T34 @ T45 @ T56

        # 提取位置和姿态
        pos = THomogeneous[:3, 3]
        T = THomogeneous[:3, :3]

        return pos, T

    def pipeendtoflange(PipeEndP, pipeEndT):
        # 该函数求解与特定喷管末端的位置PipeEndP和姿态pipeEndT相对应的
        # 法兰底面中心点的位置flangeP和姿态flangeT
        # 定义法兰在管道末端坐标系中的位置向量
        pFlangeInPipeEnd = np.array([0.146, -0.00739, -0.272])  # 扁口喷头中心到连接件法兰底面中心的向量，在喷头坐标系下

        # 定义从管道末端坐标系到法兰坐标系的变换矩阵
        flange_relate_to_pipe_end_T = np.array([[1, 0, 0],
                                                [0, 0, 1],
                                                [0, -1, 0]])   # 根据具体的喷管和法兰的朝向关系选择适当的变换矩阵，根据ppt中的位置关系

        # 计算法兰在全局坐标系中的姿态矩阵
        flangeT = np.dot(pipeEndT, flange_relate_to_pipe_end_T)

        # 计算法兰在全局坐标系中的位置向量
        flangeP = PipeEndP + np.dot(pipeEndT, pFlangeInPipeEnd)

        # 返回法兰的位置和姿态
        return flangeP, flangeT

    def directionVecToT(directVec, alpha):
        # direcVec和alpha共同确定了喷管的姿态,direcVec 喷管的指向向量,alpha为喷头沿轴线发生的转角
        # 归一化方向向量
        direct_vec = directVec / np.linalg.norm(directVec)

        # 坐标变换矩阵
        base_relative_to_local_T = np.array([[1, 0, 0],
                                             [0, 0, 1],
                                             [0, -1, 0]])

        local_relative_to_base_T = np.array([[1, 0, 0],
                                             [0, 0, -1],
                                             [0, 1, 0]])

        # 将方向向量转换到局部坐标系
        direct_vec_in_local = np.dot(base_relative_to_local_T, direct_vec)

        # 计算beta和gama
        beta = np.arctan2(direct_vec_in_local[0], direct_vec_in_local[2])
        gama = -np.arcsin(direct_vec_in_local[1])

        # 计算旋转矩阵的分量
        sr, cr = np.sin(gama), np.cos(gama)  # x轴
        sb, cb = np.sin(beta), np.cos(beta)  # y轴
        sa, ca = np.sin(alpha), np.cos(alpha)  # z轴

        # 创建旋转矩阵
        Tr = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])  # 绕x轴
        Tb = np.array([[cb, 0, sb],
                       [0, 1, 0],
                       [-sb, 0, cb]])  # 绕y轴
        Ta = np.array([[ca, -sa, 0],
                       [sa, ca, 0],
                       [0, 0, 1]])  # 绕z轴

        # 计算在局部坐标系中的总旋转矩阵
        T_in_local = np.dot(np.dot(Tb, Tr), Ta)

        # 转换回基础坐标系
        T = np.dot(local_relative_to_base_T, T_in_local)

        return T

    def invKinematicsUR10(pos, T, thetaCurrent):
        # UR10参数
        d1 = 0.128
        a1 = 0
        d4 = 0.1639
        a4 = 0
        d2 = 0
        a2 = -0.6129
        d5 = 0.1157
        a5 = 0
        d3 = 0
        a3 = -0.5716
        d6 = 0.0922
        a6 = 0

        # 提取当前角度
        thetaCurrent1, thetaCurrent2, thetaCurrent3, thetaCurrent4, thetaCurrent5, thetaCurrent6 = thetaCurrent

        # 机器人目标位置和姿态
        nx, ox, ax, px = T[0, 0], T[0, 1], T[0, 2], pos[0]
        ny, oy, ay, py = T[1, 0], T[1, 1], T[1, 2], pos[1]
        nz, oz, az, pz = T[2, 0], T[2, 1], T[2, 2], pos[2]

        # theta1求解
        m = d6 * ay - py
        n = ax * d6 - px
        theta11 = np.arctan2(m, n) - np.arctan2(d4, np.sqrt(m ** 2 + n ** 2 - d4 ** 2))
        theta12 = np.arctan2(m, n) - np.arctan2(d4, -np.sqrt(m ** 2 + n ** 2 - d4 ** 2))
        if theta11 >= 0:
            theta13 = -(2 * np.pi - theta11)
        else:
            theta13 = 2 * np.pi + theta11

        if theta12 >= 0:
            theta14 = -(2 * np.pi - theta12)
        else:
            theta14 = 2 * np.pi + theta12
        dist11 = abs(thetaCurrent1 - theta11)
        dist12 = abs(thetaCurrent1 - theta12)
        dist13 = abs(thetaCurrent1 - theta13)
        dist14 = abs(thetaCurrent1 - theta14)
        theta1 = theta11
        dist = dist11
        if dist12 < dist:
            theta1 = theta12
            dist = dist12
        if dist13 < dist:
            theta1 = theta13
            dist = dist13
        if dist14 < dist:
            theta1 = theta14

        # theta5求解
        s1 = math.sin(theta1)
        c1 = math.cos(theta1)
        theta5_val = s1 * ax - c1 * ay
        theta51 = math.acos(theta5_val)
        theta52 = -theta51
        if theta51 >= 0:
            theta53 = -(2 * math.pi - theta51)
        else:
            theta53 = 2 * math.pi + theta51

        if theta52 >= 0:
            theta54 = -(2 * math.pi - theta52)
        else:
            theta54 = 2 * math.pi + theta52
        dist51 = abs(thetaCurrent5 - theta51)
        dist52 = abs(thetaCurrent5 - theta52)
        dist53 = abs(thetaCurrent5 - theta53)
        dist54 = abs(thetaCurrent5 - theta54)
        theta5 = theta51
        dist = dist51
        if dist52 < dist:
            theta5 = theta52
            dist = dist52
        if dist53 < dist:
            theta5 = theta53
            dist = dist53
        if dist54 < dist:
            theta5 = theta54

        # theta6求解
        s5 = math.sin(theta5)
        m = nx * s1 - ny * c1
        n = ox * s1 - oy * c1
        theta61 = math.atan2(m, n) - math.atan2(s5, 0)
        if theta61 >= 0:
            theta62 = -(2 * math.pi - theta61)
        else:
            theta62 = 2 * math.pi + theta61
        dist1 = abs(thetaCurrent6 - theta61)
        dist2 = abs(thetaCurrent6 - theta62)
        if dist2 < dist1:
            theta6 = theta62
        else:
            theta6 = theta61

        # theta3求解
        s6 = math.sin(theta6)
        c6 = math.cos(theta6)
        m = d5 * ((c1 * nx + s1 * ny) * s6 + (c1 * ox + s1 * oy) * c6) - d6 * (c1 * ax + s1 * ay) + c1 * px + s1 * py
        n = d5 * (nz * s6 + oz * c6) - d6 * az + pz - d1
        theta3_val = (m ** 2 + n ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
        theta31 = math.acos(theta3_val)
        theta32 = -theta31
        if theta31 >= 0:
            theta33 = -(2 * math.pi - theta31)
        else:
            theta33 = 2 * math.pi + theta31

        if theta32 >= 0:
            theta34 = -(2 * math.pi - theta32)
        else:
            theta34 = 2 * math.pi + theta32
        dist31 = abs(thetaCurrent3 - theta31)
        dist32 = abs(thetaCurrent3 - theta32)
        dist33 = abs(thetaCurrent3 - theta33)
        dist34 = abs(thetaCurrent3 - theta34)
        theta3 = theta31
        dist = dist31
        if dist32 < dist:
            theta3 = theta32
            dist = dist32
        if dist33 < dist:
            theta3 = theta33
            dist = dist33
        if dist34 < dist:
            theta3 = theta34

            # theta2求解
        s3 = math.sin(theta3)
        c3 = math.cos(theta3)
        s2 = ((a3 * c3 + a2) * n - a3 * s3 * m) / (a2 ** 2 + a3 ** 2 + 2 * a2 * a3 * c3)
        c2 = (m + a3 * s3 * s2) / (a3 * c3 + a2)
        theta2 = math.atan2(s2, c2)

        # theta4求解
        theta41 = math.atan2(-s6 * (nx * c1 + ny * s1) - c6 * (ox * c1 + oy * s1), oz * c6 + nz * s6) - theta2 - theta3
        if theta41 >= 0:
            theta42 = -(2 * math.pi - theta41)
        else:
            theta42 = 2 * math.pi + theta41

        dist1 = abs(thetaCurrent4 - theta41)
        dist2 = abs(thetaCurrent4 - theta42)
        if dist2 < dist1:
            theta4 = theta42
        else:
            theta4 = theta41

        return [theta1, theta2, theta3, theta4, theta5, theta6]