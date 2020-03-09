"""

Histogram Filter 2D localization example


In this simulation, x,y are unknown, yaw is known.

Initial position is not needed.

author: Atsushi Sakai (@Atsushi_twi)

"""


'''
https://zhuanlan.zhihu.com/p/96859841
'''


import copy
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter #多维高斯滤波器
from scipy.stats import norm

# Parameters
EXTEND_AREA = 10.0  # [m] grid map extended length
SIM_TIME = 50.0  # simulation time [s]
DT = 0.1  # time tick [s]
MAX_RANGE = 10.0  # maximum observation range
MOTION_STD = 1.0  # standard deviation for motion gaussian distribution
RANGE_STD = 3.0  # standard deviation for observation gaussian distribution

# grid map param
XY_RESO = 0.5  # xy grid resolution
MINX = -15.0
MINY = -5.0
MAXX = 15.0
MAXY = 25.0

# simulation parameters
NOISE_RANGE = 2.0  # [m] 1σ range noise parameter
NOISE_SPEED = 0.5  # [m/s] 1σ speed noise parameter

show_animation = True


class GridMap():

    def __init__(self):
        self.data = None
        self.xy_reso = None
        self.minx = None
        self.miny = None
        self.maxx = None
        self.maxx = None
        self.xw = None
        self.yw = None
        self.dx = 0.0  # movement distance
        self.dy = 0.0  # movement distance


def histogram_filter_localization(grid_map, u, z, yaw):
    grid_map = motion_update(grid_map, u, yaw)#根据带噪声的速度角速度观测值和车辆的yaw角更新栅格地图数据    先运动更新，即利用运动模型将每一个栅格点的值进行移动

    grid_map = observation_update(grid_map, z, RANGE_STD)#                                             再观测更新！！！！

    return grid_map

#对每一个栅格进行操作
def calc_gaussian_observation_pdf(gmap, z, iz, ix, iy, std):
    # predicted range
    x = ix * gmap.xy_reso + gmap.minx # 由栅格地图坐标转换为局部 x y 坐标， 即为  这个栅格  的实际坐标
    y = iy * gmap.xy_reso + gmap.miny
    d = math.hypot(x - z[iz, 1], y - z[iz, 2]) # 计算观测点和这个栅格的实际距离

    # likelihood，1.0-只是因为概率值越小，则栅格地图数据越大
    #因为我们只能测得与landmark的距离d，而不知道它的具体方位，则只要某个栅格与观测点的实际距离接近这个d，则它的概率就越大
    pdf = (1.0 - norm.cdf(abs(d - z[iz, 0]), 0.0, std)) # norm.cdf(x,μ，σ) 返回x处的正太分布函数值，这里即设置了一个x~（μ，σ)的正太分布，这里得到的是这个栅格与观测点实际距离与测量得到的距离之差的绝对值在这个正态分布里面的概率值

    return pdf

#跟还有观测值更新栅格地图数据
def observation_update(gmap, z, std):
    for iz in range(z.shape[0]): # 遍历每个观测点，每个观测点的值都要对每个栅格的概率进行更新，因为是高斯分布，所以每个观测值对于每个栅格的概率累乘，仍然是高斯分布

        for ix in range(gmap.xw): # 遍历栅格地图每个点
            for iy in range(gmap.yw):

                gmap.data[ix][iy] *= calc_gaussian_observation_pdf(#每个观测值的概率都要累乘上去
                    gmap, z, iz, ix, iy, std)

    gmap = normalize_probability(gmap)

    return gmap


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([v, yaw_rate]).reshape(2, 1)
    return u

#x=[x y yaw v]' u=[v,w]'
def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def draw_heat_map(data, mx, my):
    maxp = max([max(igmap) for igmap in data])
    plt.pcolor(mx, my, data, vmax=maxp, cmap=plt.cm.get_cmap("Greens"))
    plt.axis("equal")


def observation(xTrue, u, RFID):
    xTrue = motion_model(xTrue, u)#根据此时的状态和速度角速度计算下一时刻的状态

    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            # add noise to range observation
            dn = d + np.random.randn() * NOISE_RANGE
            zi = np.array([dn, RFID[i, 0], RFID[i, 1]])
            z = np.vstack((z, zi))#保存在规定距离范围内的目标rf与车辆的距离（加了噪声，表示观测误差），以及相应的rf的真实位置

    # add noise to speed
    ud = u[:, :]
    ud[0] += np.random.randn() * NOISE_SPEED#速度和角速度也要加噪声，表示这两个量的测量也有噪声

    return xTrue, z, ud

#概率归一化
def normalize_probability(gmap):
    sump = sum([sum(igmap) for igmap in gmap.data])

    for ix in range(gmap.xw):
        for iy in range(gmap.yw):
            gmap.data[ix][iy] /= sump

    return gmap


def init_gmap(xy_reso, minx, miny, maxx, maxy):
    grid_map = GridMap()

    grid_map.xy_reso = xy_reso
    grid_map.minx = minx
    grid_map.miny = miny
    grid_map.maxx = maxx
    grid_map.maxy = maxy
    grid_map.xw = int(round((grid_map.maxx - grid_map.minx) / grid_map.xy_reso))#x方向的栅格地图格数
    grid_map.yw = int(round((grid_map.maxy - grid_map.miny) / grid_map.xy_reso))#y方向的栅格地图格数

    grid_map.data = [[1.0 for _ in range(grid_map.yw)] for _ in range(grid_map.xw)]#二维数组，保存每个栅格地图的值
    grid_map = normalize_probability(grid_map)

    return grid_map


def map_shift(grid_map, x_shift, y_shift):
    tgmap = copy.deepcopy(grid_map.data)

    for ix in range(grid_map.xw):
        for iy in range(grid_map.yw):
            nix = ix + x_shift
            niy = iy + y_shift

            if 0 <= nix < grid_map.xw and 0 <= niy < grid_map.yw:#如果[ix,iy]移动后的坐标[nix,niy]仍然在栅格地图范围以内，则后面栅格坐标[nix,niy]处的data为[ix,iy]处的data
                grid_map.data[ix + x_shift][iy + y_shift] = tgmap[ix][iy]

    return grid_map

# grid_map是网格地图，u=(v,w)是车辆运动的控制参数，yaw是车辆朝向
def motion_update(grid_map, u, yaw):
    grid_map.dx += DT * math.cos(yaw) * u[0]#相对于最初的位置 在xy方向上的位移
    grid_map.dy += DT * math.sin(yaw) * u[0]

    x_shift = grid_map.dx // grid_map.xy_reso# // 整数除法,返回不大于结果的一个最大的整数，相对于最初位置在xy方向走的格数
    y_shift = grid_map.dy // grid_map.xy_reso

    #例如向右移动了两格，则栅格地图最左边的一列的data肯定不会改变
    if abs(x_shift) >= 1.0 or abs(y_shift) >= 1.0:  # map should be shifted，表示点移动的距离超过了一个栅格，则需要更新data，否则栅格的data不需要改变
        grid_map = map_shift(grid_map, int(x_shift), int(y_shift))
        grid_map.dx -= x_shift * grid_map.xy_reso#将移动的整数倍xy_reso的dx dy减去，只保留小数部分，这样可以减小一些运动累加误差，比如这次前进了1.5倍xy_reso，则栅格数据的移动只会移动1格，但是另外的0.5倍必须保留，下次运动一起累加上去
        grid_map.dy -= y_shift * grid_map.xy_reso
    # MOTION_STD是车辆运动不确定性的标准差
    grid_map.data = gaussian_filter(grid_map.data, sigma=MOTION_STD)#用高维滤波器，比如某个点的数据为1.0,则周围根据这个点的值做一个高斯模糊，周围的点可能变为0.8,0.7等等

    return grid_map

#为了栅格地图的可视化
def calc_grid_index(gmap):
    # #np.mgrid[ 第1维，第2维 ，第3维 ， …] 返回多维结构
    mx, my = np.mgrid[slice(gmap.minx - gmap.xy_reso / 2.0, gmap.maxx + gmap.xy_reso / 2.0, gmap.xy_reso),#slice() 函数实现切片对象，主要用在切片操作函数里的参数传递。slice(start, stop[, step])
                      slice(gmap.miny - gmap.xy_reso / 2.0, gmap.maxy + gmap.xy_reso / 2.0, gmap.xy_reso)]# - gmap.xy_reso / 2.0只是为了画的图比栅格地图更宽一点
    print("mx=",mx)
    print("my=",my)
    return mx, my


def main():
    print(__file__ + " start!!")

    # RF_ID positions [x, y]， landmark的位置坐标，已知条件，后面根据观测到的landmark来更新自身的位置
    RF_ID = np.array([[10.0, 0.0],
                      [10.0, 10.0],
                      [0.0, 15.0],
                      [-5.0, 20.0]])

    time = 0.0

    xTrue = np.zeros((4, 1)) #[x y yaw v]'
    grid_map = init_gmap(XY_RESO, MINX, MINY, MAXX, MAXY)#建立栅格地图，确定origin以及resolution，每个栅格的值初始化为1.0
    mx, my = calc_grid_index(grid_map)  # for grid map visualization

    while SIM_TIME >= time:
        time += DT
        print("Time:", time)

        u = calc_input()#计算输入的[v w]'

        yaw = xTrue[2, 0]  # Orientation is known 初始时车辆朝向已知，必须已知吗？？？
        # print("yaw=",yaw)
        xTrue, z,  ud= observation(xTrue, u, RF_ID)#根据此时的状态和运动模型及速度角速度得到下一时刻的状态，并且根据观测范围得到范围以内的目标rf的
                                                   # 距离（加了观测噪声），以及速度角速度的观测值（加了噪声）

        grid_map = histogram_filter_localization(grid_map, u, z, yaw)#更新栅格地图

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            draw_heat_map(grid_map.data, mx, my)
            plt.plot(xTrue[0, :], xTrue[1, :], "xr")
            plt.plot(RF_ID[:, 0], RF_ID[:, 1], ".k")
            for i in range(z.shape[0]):
                plt.plot([xTrue[0, :], z[i, 1]], [
                    xTrue[1, :], z[i, 2]], "-k")
            plt.title("Time[s]:" + str(time)[0: 4])
            plt.pause(0.001)

    print("Done")


if __name__ == '__main__':
    main()
