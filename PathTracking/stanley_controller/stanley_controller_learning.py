'''
https://blog.csdn.net/renyushuai900/article/details/98460758?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task
http://element-ui.cn/news/show-16156.aspx
Pure pursuit是以后轴中心为基准点计算几何学公式，而Stanley是基于前轴中心为基准点计算几何学公式的。

步骤：
	1.确定初始状态（x,y,yaw,v)
	2.根据纵向控制器得到加速度a，这里我们就用一个简单的P控制器，用速度差确定
	3.利用Stanley_controller得到前轮转角
	  首先搜索与现在状态最邻近的路径上的点，如果这个target_index比上一的小，则还是用上一次的，如果数字大于路径点的总数，则用终点作为目标点,然后用速度生成前视距离d
	  根据角度关系计算前轮转角delta,这里需要注意，转向的角度应该限制在一个范围内
	  利用dt和现在的前轮转角更新车辆的状态
	  返回第二步计算，直到迭代时间到达，或者目标追踪点为终点（应该是目标点与最终点的距离小于一个阈值）
'''

import numpy as np
import math
import matplotlib.pyplot as plt
import sys
sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise


k = 0.5  # control gain,预瞄点与轨迹最近点距离
Kp = 1.0  # speed proportional gain，纵向控制速度增益
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle
max_a=3.0 #最大加速度
max_w=np.radians(60.0)#最大角速度

global print_time
print_time=0

class stanley_controller:
	def __init__(self,cx,cy,target_speed,x,y,yaw,v):
		self.cx=cx# 跟踪路径
		self.cy=cy
		self.target_speed=target_speed
		self.init_x=x
		self.init_y=y
		self.init_yaw=yaw
		self.init_v=v

	class State:
		def __init__(self,x,y,yaw,v):
			self.x=x
			self.y=y
			self.yaw=yaw
			self.v=v

		def update(self,delta,a):
			delta=np.clip(delta,-1*max_steer,max_steer)#将前轮转角限制在(-1*max_steer,max_steer)范围内!!!!!!
			a=np.clip(a,-max_a,max_a)
			w=self.v/L*math.tan(delta)
			w=np.clip(w,-max_w,max_w)
			self.x=self.x+self.v*math.cos(self.yaw)*dt
			self.y=self.y+self.v*math.sin(self.yaw)*dt
			self.yaw=self.yaw+w*dt
			self.yaw=self.normalize_angle(self.yaw)
			self.v=self.v+a*dt

		@staticmethod
		def normalize_angle(angle):
			# Normalize an angle to [-pi, pi].
			return (angle+math.pi)%(2*math.pi)-math.pi
		# @staticmethod
		# def normalize_angle_0_2pi(angle):
		# 	return (angle+math.pi)%(2*math.pi)
	class States:
		def __init__(self):
			self.t=[]
			self.x=[]
			self.y=[]
			self.yaw=[]
			self.v=[]
		def append(self,t,state):
			self.t.append(t)
			self.x.append(state.x)
			self.y.append(state.y)
			self.yaw.append(state.yaw)
			self.v.append(state.v)

	#纵向控制器，简单的P控制，得到纵向控制量：加速度
	def vertical_control(self,expect_vel,actual_vel):
		return Kp*(expect_vel-actual_vel)

	#横向控制，输入量为上一时刻的目标点的index
	def stanley_control(self,state,pind):
		# 返回最近点的index，最近点切线角度，横向偏差值，离最近点的距离
		ind,tangent_direction,error_front_axle,min_dist=self.cal_nearest_point(state)
		if ind<pind:
			ind=pind#????有可能此时的转向还没有转过来，没有能朝向前进的方向，如果直接用ind，则车辆可能发生倒车

		# theta_e corrects the heading error
		theta_e = self.normalize_angle(tangent_direction - state.yaw)#!!!!!!!!!!!!!!
		# theta_d corrects the cross track error
		theta_d = math.atan2(k * error_front_axle, state.v)
		# Steering control
		# delta = theta_d
		delta = theta_e + theta_d


		# global print_time
		# print_time+=1
		# if print_time%20==0:
		# 	print("tangent_direction=",round(tangent_direction,3), "yaw=",round(state.yaw,3),"theta_e=",round(theta_e,3)," theta_d=",round(theta_d,3)," delta=",round(delta,3))


		# delta=self.normalize_angle(tangent_direction+math.atan2(k*theta_e,state.v))

		# if state.v<0:
		# 	delta=math.pi-delta

		return ind,delta,min_dist
	#返回最近点的index，最近点切线角度，横向偏差值，离最近点的距离
	def cal_nearest_point(self,state):


		fx = state.x + L * np.cos(state.yaw)#将后轮中心转换到前轮中心计算最近点
		fy = state.y + L * np.sin(state.yaw)

		# Search nearest point index
		dx = [fx - icx for icx in cx]
		dy = [fy - icy for icy in cy]

		#搜索目标路径上与此时位置最近的点
		# dx=[ state.x-i for i in cx]
		# dy=[ state.y-i for i in cy]
		dist=[math.hypot(i,j) for (i,j) in zip(dx,dy)]
		min_dist=min(dist)#横向位置偏差
		nearest_index=dist.index(min_dist)

		# Project RMS error onto front axle vector
		front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
							-np.sin(state.yaw + np.pi / 2)]
		error_front_axle = np.dot([dx[nearest_index], dy[nearest_index]], front_axle_vec)  #在曲线下方的为正值，曲线上方的为负值,在下面确实要左打方向，delta为正值，越接近路径绝对值越小
		# print(error_front_axle)
		# alpha = math.atan2(dy[nearest_index], dx[nearest_index])
		# error_front_axle = np.sign(np.sin(alpha-state.yaw))*dist[nearest_index]



		# error=min_dist*math.atan2(dy[nearest_index],dx[nearest_index])
		if nearest_index==len(self.cx)-1:
			tangent_direction=math.atan2(cy[nearest_index]-cy[nearest_index-1],cx[nearest_index]-cx[nearest_index-1])
		else:
			tangent_direction=math.atan2(cy[nearest_index+1]-cy[nearest_index],cx[nearest_index+1]-cx[nearest_index])

		return nearest_index,tangent_direction,error_front_axle,min_dist

	@staticmethod
	def normalize_angle(angle):

  
		# while angle > np.pi:
		# 	angle -= 2.0 * np.pi

		# while angle < -np.pi:
		# 	angle += 2.0 * np.pi

		# return angle

		# Normalize an angle to [-pi, pi].
		return (angle+math.pi)%(2*math.pi)-math.pi

	# @staticmethod
	# def normalize_angle_0_2pi(angle):
	# 	return (angle+math.pi)%(2*math.pi)

	def main(self):
		state=self.State(self.init_x,self.init_y,self.init_yaw,self.init_v)#初始状态
		states=self.States()
		t=0.0
		states.append(t,state)
		max_iter_T=500.0
		ind,_,_,min_dist=self.cal_nearest_point(state)
		# print(ind,"222")

		dist=[min_dist]

		while  t<max_iter_T:
			if ind==len(self.cx)-1:
				break
			a=self.vertical_control(self.target_speed,state.v)
			ind,delta,min_dist=self.stanley_control(state,ind)
			t+=dt


			dist.append(min_dist)

			state.update(delta,a)
			states.append(t,state)


			plt.cla()#删除上一幅图axes，保留figure

			# for stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect('key_release_event',
				lambda event: [exit(0) if event.key == 'escape' else None])

			# self.plot_arrow(state.x, state.y, state.yaw)
			plt.plot(cx, cy, "-r", label="course")
			plt.plot(states.x, states.y, "-b", label="trajectory")
			plt.plot(cx[ind], cy[ind], "xg", label="target")
			plt.axis("equal")
			plt.grid(True)
			plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
			plt.pause(0.001)

		plt.cla()
		plt.plot(cx, cy, ".r", label="course")
		plt.plot(states.x, states.y, "-b", label="trajectory")
		plt.legend()
		plt.xlabel("x[m]")
		plt.ylabel("y[m]")
		plt.axis("equal")
		plt.grid(True)

		plt.subplots(1)
		plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
		plt.xlabel("Time[s]")
		plt.ylabel("Speed[km/h]")
		plt.grid(True)

		plt.subplots(1)
		plt.plot(states.t, dist, "-r")
		plt.xlabel("Time[s]")
		plt.ylabel("dist_error[/m]")
		plt.grid(True)
		plt.title("target_speed[km/h]:" + str(self.target_speed * 3.6)[:4])

		plt.show()

	# def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
	# 	"""
	# 	Plot arrow
	# 	"""

	# 	if not isinstance(x, float):
	# 		for ix, iy, iyaw in zip(x, y, yaw):
	# 			plot_arrow(ix, iy, iyaw)
	# 	else:
	# 		plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
	# 					fc=fc, ec=ec, head_width=width, head_length=width)
	# 		plt.plot(x, y)


if __name__ == '__main__':
	cx = np.arange(0, 250, 0.1)
	cy = [math.sin(ix / 25.0) * ix / 2.0 for ix in cx]
	# plt.plot(cx,cy,'-b')
	target_speed = 60.0 / 3.6  # [m/s]


 #  target course
	# ax = [0.0, 100.0, 100.0, 50.0, 60.0]
	# ay = [0.0, 0.0, -30.0, -20.0, 0.0]

	# cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
	# 	ax, ay, ds=0.1)
	plt.plot(cx,cy,'-b')

	
	x=-1
	y=-3
	yaw=0.9
	v=0.0
	stanley_controller=stanley_controller(cx,cy,target_speed,x,y,yaw,v)
	stanley_controller.main()

	# initial state