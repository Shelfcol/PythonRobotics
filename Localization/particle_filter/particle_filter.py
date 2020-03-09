"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

这里只是定位，通过粒子滤波对机器人的位姿进行实时估计，最终会发现比传感器测量的要更准



"""

import math

import matplotlib.pyplot as plt
import numpy as np

# Estimation parameter of PF
Q = np.diag([0.2]) ** 2  # range error
# print(Q)
R = np.diag([2.0, np.deg2rad(40.0)]) ** 2  # input error

#  Simulation parameter
Q_sim = np.diag([0.2]) ** 2
R_sim = np.diag([1.0, np.deg2rad(30.0)]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 150.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range

# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

show_animation = True

#获取2*1的速度角速度矩阵
def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T#获得转置矩阵 (2,1)矩阵
    # print(u.shape)
    return u
#输入 此状态的真实值 此时的航迹推算位姿 此时的速度角速度值 目标rf的位置
#输出 下一时刻的真实状态，观测到的landmark的加了噪声的距离(仪器测量得到) 以及landmark的真实坐标，航迹推算值（加了噪声的速度角速度），加了噪声的速度角速度（表示速度角速度的测量值）
def observation(x_true, xd, u, rf_id):
    x_true = motion_model(x_true, u)#根据运动方程求出下一时刻的真实状态，作为对照

    # add noise to gps x-y
    z = np.zeros((0, 3))
    print(z.shape)

    #待观测的目标位置已知，求出此时车的真实位置与rf目标的距离，然后加上高斯噪声，代表车上传感器测量的rf目标的测量值（含噪声），并且输出对应的距离，对应目标的xy坐标
    for i in range(len(rf_id[:, 0])):

        dx = x_true[0, 0] - rf_id[i, 0]
        dy = x_true[1, 0] - rf_id[i, 1]
        d = math.hypot(dx, dy)#求出此时的真实位置与每个定位点（辅助定位的）的距离
        if d <= MAX_RANGE: #如果超过了测量范围，则不计入测量
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise 为每个距离加上噪声，可以视为传感器的测量噪声
            zi = np.array([[dn, rf_id[i, 0], rf_id[i, 1]]]) #zi存储于每个定位点的测量距离（带噪声）以及其真实的坐标值 1*3
            print(zi.shape)
            z = np.vstack((z, zi))#将素有的测量堆叠上去

    # add noise to input  给速度和角速度加噪声，表示这两者的测量有误差
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[ud1, ud2]]).T

    #得到航迹推算结果
    xd = motion_model(xd, ud) #根据上一时刻的航迹推算结果以及加了噪声的速度角速度得到下一时刻的状态

    return x_true, z, xd, ud

#用的是最简单的运动方程，根据此时的状态，以及速度角速度，求出下一时刻的状态
def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])
    # print(x.shape)
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x

#根据此时的x和sigma生成对应含有高斯噪声的x值
def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(x_est, px, pw):
    """
    calculate covariance matrix
    see ipynb doc
    """
    cov = np.zeros((3, 3))
    n_particle = px.shape[1]
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)[0:3]
        cov += pw[0, i] * dx @ dx.T
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov

#输入 粒子群 粒子权重 状态观测值（包含rf目标的距离测量值和它的真实位置） 速度角速度的观测值
#输出 粒子群加权得到的状态预测值 协方差 粒子群 粒子权重
def pf_localization(px, pw, z, u):
    """
    Localization with Particle filter
    虽然px中存储了速度角速度，但是每次使用的还是只是上次的xy值，而速度角速度都是这一次的测量值
    """

    for ip in range(NP): #对每个粒子的x y yaw v进行粒子滤波操作
        x = np.array([px[:, ip]]).T
        w = pw[0, ip]

        #  Predict with random input sampling 输入的速度角速度已经加了高斯噪声，这里还需要给粒子加高斯噪声
        ud1 = u[0, 0] + np.random.randn() * R[0, 0] ** 0.5
        ud2 = u[1, 0] + np.random.randn() * R[1, 1] ** 0.5
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud)#根据粒子得到的速度角速度预测下一时刻的状态

        #  Calc Importance Weight，利用测量的目标rf的观测值和pf预测的目标rf的距离来更新粒子权重
        for i in range(len(z[:, 0])):
            dx = x[0, 0] - z[i, 1] # 利用预测的车辆位置与目标rf的真实值!!!!!求得距离，真实距离怎么求得？？？？？？？
            dy = x[1, 0] - z[i, 2]
            pre_z = math.hypot(dx, dy)
            dz = pre_z - z[i, 0]#得到预测的车辆与目标rf的距离和实际目标rf的距离测量值做差
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))#用预测得到的距离与测量得到的距离的高斯值作为权重

        px[:, ip] = x[:, 0] #更新这个粒子的值（x y yaw v）
        pw[0, ip] = w #更新粒子的权重

    pw = pw / pw.sum()  # normalize 归一化

    #这里没有进行重采样就直接用粒子和权重得到粒子预测状态，可能有点不妥？？？？？？？？
    x_est = px.dot(pw.T) # 4*1  粒子值的加权平均得到粒子估计值(x y yaw v),用这个估计值与航迹推算结果进行对比，结果表明更好
    # print(x_est.shape)
    p_est = calc_covariance(x_est, px, pw)#根据粒子加权得到的状态值和所有粒子及权值按照论文计算协方差，但是这个协方差并没有使用，只是用来画图

    #有效粒子数Neff衡量粒子权值的退化程度
    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number，计算有效粒子数 1/（权值的平方和）
    if N_eff < NTh:#如果有效粒子数太少，则进行重采样
        px, pw = re_sampling(px, pw)
    return x_est, p_est, px, pw

#重采样之后所有粒子权重一样
def re_sampling(px, pw):
    """
    low variance re-sampling
    """

    w_cum = np.cumsum(pw) #每个位置的权值是前面权值和[1,3,5]->[1,4,9]，即为轮盘采样的方法
    base = np.arange(0.0, 1.0, 1 / NP)#返回0.0-1.0步长为1/NP的数列 
    re_sample_id = base + np.random.uniform(0, 1 / NP)#加上噪声，形成均匀分布的随机采样值
    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind) #存储重采样后的id

    px = px[:, indexes]
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw


def plot_covariance_ellipse(x_est, p_est):  # pragma: no cover
    p_xy = p_est[0:2, 0:2]
    eig_val, eig_vec = np.linalg.eig(p_xy)

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # eig_val[big_ind] or eiq_val[small_ind] were occasionally negative numbers extremely
    # close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try:
        a = math.sqrt(eig_val[big_ind])
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eig_val[small_ind])
    except ValueError:
        b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eig_vec[big_ind, 1], eig_vec[big_ind, 0])
    Rot = np.array([[math.cos(angle), -math.sin(angle)],
                    [math.sin(angle), math.cos(angle)]])
    fx = Rot.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + x_est[0, 0]).flatten()
    py = np.array(fx[1, :] + x_est[1, 0]).flatten()
    plt.plot(px, py, "--r")
    # plt.show()


def main():
    print(__file__ + " start!!")

    time = 0.0

    # RF_ID positions [x, y]
    rf_id = np.array([[10.0, 0.0],
                      [10.0, 10.0],
                      [0.0, 15.0],
                      [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    x_est = np.zeros((4, 1))
    x_true = np.zeros((4, 1))

    #粒子及其权重的初始化
    px = np.zeros((4, NP))  # Particle store 粒子群，x y yaw v各对应一群粒子
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight  粒子权重均匀分布
    x_dr = np.zeros((4, 1))  # Dead reckoning

    # history
    h_x_est = x_est
    h_x_true = x_true
    h_x_dr = x_true

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        x_true, z, x_dr, ud = observation(x_true, x_dr, u, rf_id)

        x_est, PEst, px, pw = pf_localization(px, pw, z, ud)

        # store data history
        h_x_est = np.hstack((h_x_est, x_est))
        h_x_dr = np.hstack((h_x_dr, x_dr))
        h_x_true = np.hstack((h_x_true, x_true))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            for i in range(len(z[:, 0])):
                plt.plot([x_true[0, 0], z[i, 1]], [x_true[1, 0], z[i, 2]], "-k")
            plt.plot(rf_id[:, 0], rf_id[:, 1], "*k")
            plt.plot(px[0, :], px[1, :], ".r")
            plt.plot(np.array(h_x_true[0, :]).flatten(),
                     np.array(h_x_true[1, :]).flatten(), "-b")#航迹推算结果
            plt.plot(np.array(h_x_dr[0, :]).flatten(),
                     np.array(h_x_dr[1, :]).flatten(), "-g")
            plt.plot(np.array(h_x_est[0, :]).flatten(),
                     np.array(h_x_est[1, :]).flatten(), "-r")
            # plot_covariance_ellipse(x_est, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
