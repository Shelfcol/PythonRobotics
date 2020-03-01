"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

https://blog.csdn.net/clong139/article/details/83014865
博客里的公式与此中所用公式一样

Dubins曲线是满足曲率约束和规定的始末端切线方向条件下连接二维平面两个点的最短路径，车辆只能前进
人和路径可以由最大曲率（最小转弯半径）和直线段组成（两点之间的路径首先得存在）
连接两点的最短路径将通过最大曲率的曲的圆弧和直线段的构成

Vx=Vcos(theta)
Vy=Vsin(theta)
w

最佳路径类型可以用与右转（R），左转（L）或驾驶’直（S）’的汽车类比来描述。
最佳路径总是至少有六种类型之一：RSR，RSL，LSR，LSL，RLR，LRL。每次根据起始点对每条路径的代价进行计算比较，最终选出最佳的路径，其中要排除不满足条件的，比如有障碍物

问题：Dubins间隔问题，即在初始点和终点指定了航向的间隔。 
也就是说路径在初始点和终点处的切线方向被限制在指定的间隔内

"""
import math

import matplotlib.pyplot as plt
import numpy as np

show_animation = True

#将角度转换到0-2pi
def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def left_straight_left(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))#中间段距离平方
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)#转换到0-2pi
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)

    return t, p, q, mode


def right_straight_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)

    return t, p, q, mode


def left_straight_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    return t, p, q, mode


def right_straight_left(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode


def right_left_right(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q, mode


def left_right_left(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(end_x, end_y, end_yaw, curvature, step_size):
    dx = end_x
    dy = end_y
    D = math.hypot(dx, dy)
    d = D * curvature # 将转弯半径转为r = 1，这样处理可以直接使用弧度来表示路径的长度，可大大简化后续的计算工作

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(- theta)
    beta = mod2pi(end_yaw - theta)

    planners = [left_straight_left, right_straight_right, left_straight_right, right_straight_left, right_left_right,
                left_right_left]

    best_cost = float("inf")
    bt, bp, bq, best_mode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if best_cost > cost:
            bt, bp, bq, best_mode = t, p, q, mode
            best_cost = cost
    lengths = [bt, bp, bq]

    px, py, pyaw, directions = generate_local_course(
        sum(lengths), lengths, best_mode, curvature, step_size)

    return px, py, pyaw, best_mode, best_cost


def interpolate(ind, length, mode, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions):
    if mode == "S":
        path_x[ind] = origin_x + length / max_curvature * math.cos(origin_yaw)
        path_y[ind] = origin_y + length / max_curvature * math.sin(origin_yaw)
        path_yaw[ind] = origin_yaw
    else:  # curve
        ldx = math.sin(length) / max_curvature
        ldy = 0.0
        if mode == "L":  # left turn
            ldy = (1.0 - math.cos(length)) / max_curvature
        elif mode == "R":  # right turn
            ldy = (1.0 - math.cos(length)) / -max_curvature
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        path_x[ind] = origin_x + gdx
        path_y[ind] = origin_y + gdy

    if mode == "L":  # left turn
        path_yaw[ind] = origin_yaw + length
    elif mode == "R":  # right turn
        path_yaw[ind] = origin_yaw - length

    if length > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return path_x, path_y, path_yaw, directions


def dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, c, step_size=0.1):
    """
    Dubins path plannner

    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]

    output:
        px
        py
        pyaw
        mode

    """

    ex = ex - sx
    ey = ey - sy

    lex = math.cos(syaw) * ex + math.sin(syaw) * ey
    ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
    leyaw = eyaw - syaw

    lpx, lpy, lpyaw, mode, clen = dubins_path_planning_from_origin(
        lex, ley, leyaw, c, step_size)

    #将坐标系转换回去
    px = [math.cos(-syaw) * x + math.sin(-syaw)
          * y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw)
          * y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]

    return px, py, pyaw, mode, clen


def generate_local_course(total_length, lengths, mode, max_curvature, step_size):
    n_point = math.trunc(total_length / step_size) + len(lengths) + 4

    path_x = [0.0 for _ in range(n_point)]
    path_y = [0.0 for _ in range(n_point)]
    path_yaw = [0.0 for _ in range(n_point)]
    directions = [0.0 for _ in range(n_point)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    ll = 0.0

    for (m, l, i) in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        # set origin state
        origin_x, origin_y, origin_yaw = path_x[ind], path_y[ind], path_yaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = - d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            path_x, path_y, path_yaw, directions = interpolate(
                ind, pd, m, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        path_x, path_y, path_yaw, directions = interpolate(
            ind, l, m, max_curvature, origin_x, origin_y, origin_yaw, path_x, path_y, path_yaw, directions)

    if len(path_x) <= 1:
        return [], [], [], []

    # remove unused data
    while len(path_x) >= 1 and path_x[-1] == 0.0:
        path_x.pop()
        path_y.pop()
        path_yaw.pop()
        directions.pop()

    return path_x, path_y, path_yaw, directions


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    print("Dubins path planner sample start!!")

    start_x = 3.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 0.2

    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y, start_yaw,
                                                    end_x, end_y, end_yaw, curvature)

    if show_animation:
        plt.plot(px, py, label="final course " + "".join(mode))

        # plotting
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()


if __name__ == '__main__':
    main()
