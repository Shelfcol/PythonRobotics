"""

Object shape recognition with circle fitting

author: Atsushi Sakai (@Atsushi_twi)

最小二乘拟合圆
https://blog.csdn.net/sinat_21107433/article/details/80877704

"""

import matplotlib.pyplot as plt
import math
import random
import numpy as np

show_animation = True


def circle_fitting(x, y):
    """
    Circle Fitting with least squared
        input: point x-y positions
        output  cxe x center position
                cye y center position
                re  radius of circle
                error: prediction error
    """

    sumx = sum(x)#横坐标之和
    sumy = sum(y)
    sumx2 = sum([ix ** 2 for ix in x])#每个x平方和
    sumy2 = sum([iy ** 2 for iy in y])
    sumxy = sum([ix * iy for (ix, iy) in zip(x, y)])

    F = np.array([[sumx2, sumxy, sumx],
                  [sumxy, sumy2, sumy],
                  [sumx, sumy, len(x)]])

    G = np.array([[-sum([ix ** 3 + ix * iy ** 2 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 * iy + iy ** 3 for (ix, iy) in zip(x, y)])],
                  [-sum([ix ** 2 + iy ** 2 for (ix, iy) in zip(x, y)])]])

    T = np.linalg.inv(F).dot(G)

    cxe = float(T[0] / -2)
    cye = float(T[1] / -2)
    re = math.sqrt(cxe**2 + cye**2 - T[2])

    error = sum([np.hypot(cxe - ix, cye - iy) - re for (ix, iy) in zip(x, y)])

    return (cxe, cye, re, error)


def get_sample_points(cx, cy, cr, angle_reso):
    x, y, angle, r = [], [], [], []

    # points sampling
    for theta in np.arange(0.0, 2.0 * math.pi, angle_reso):#遍历0-2π
        nx = cx + cr * math.cos(theta)
        ny = cy + cr * math.sin(theta)
        nangle = math.atan2(ny, nx)
        nr = math.hypot(nx, ny) * random.uniform(0.95, 1.05)#0.95~1.05的随机值

        x.append(nx)#圆上的点
        y.append(ny)
        angle.append(nangle)#圆上点原点连线的角度
        r.append(nr)#加了噪声的点到原点的距离

    # ray casting filter
    rx, ry = ray_casting_filter(x, y, angle, r, angle_reso)

    return rx, ry

'''
https://www.jianshu.com/p/ba03c600a557
光线投射法判断点是否在多边形的内部：从这个点引出一根“射线”，与多边形的任意若干条边相交，累计相交的边的数目，如果是奇数，那么点就在多边形内，否则点就在多边形外。
射线法的关键是正确计算  射线  与  每条边  是否相交。并且规定线段与射线重叠或者射线经过线段下端点属于不相交。首先排除掉不相交的情况
'''

def ray_casting_filter(xl, yl, thetal, rangel, angle_reso):
    rx, ry = [], []
    rangedb = [float("inf") for _ in range(
        int(math.floor((math.pi * 2.0) / angle_reso)) + 1)]

    for i, _ in enumerate(thetal):#enumerate() 函数用于将一个可遍历的数据对象(如列表、元组或字符串)组合为一个索引序列，  同时列出   数据和数据下标，一般用在 for 循环当中
        angleid = math.floor(thetal[i] / angle_reso)

        if rangedb[angleid] > rangel[i]:
            rangedb[angleid] = rangel[i]

    for i, _ in enumerate(rangedb):
        t = i * angle_reso
        if rangedb[i] != float("inf"):
            rx.append(rangedb[i] * math.cos(t))
            ry.append(rangedb[i] * math.sin(t))

    return rx, ry


def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)


def main():

    # simulation parameters
    simtime = 15.0  # simulation time
    dt = 1.0  # time tick

    cx = -2.0  # initial x position of obstacle
    cy = -8.0  # initial y position of obstacle
    cr = 1.0  # obstacle radious
    theta = np.deg2rad(30.0)  # obstacle moving direction
    angle_reso = np.deg2rad(3.0)  # sensor angle resolution

    time = 0.0
    while time <= simtime:
        time += dt

        cx += math.cos(theta)
        cy += math.cos(theta)

        x, y = get_sample_points(cx, cy, cr, angle_reso)

        ex, ey, er, error = circle_fitting(x, y)
        print("Error:", error)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.axis("equal")
            plt.plot(0.0, 0.0, "*r")
            plot_circle(cx, cy, cr)
            plt.plot(x, y, "xr")
            plot_circle(ex, ey, er, "-r")
            plt.pause(dt)

    print("Done")


if __name__ == '__main__':
    main()
