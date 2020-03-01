'''
https://blog.csdn.net/yuxuan20062007/article/details/80914439
每次dt内状态（x，y，yaw，v）更新是利用这次的速度，以及与R得到的角速度，前轮转角delta，更新yaw角和x，y，v

步骤：
	1.确定初始状态（x,y,yaw,v)
	2.根据纵向控制器得到加速度a，这里我们就用一个简单的P控制器，用速度差确定
	3.利用pure_pursuit得到前轮转角
	  首先搜索与现在状态最邻近的路径上的点，然后用速度生成前视距离，从这个最近邻的点向后面的路径点进行搜索，
	  将每段的距离加起来，当加起来的距离大于生成的前视距离的时候，表示找到了最佳的目标点位置
	  如果这个target_index比上一的小，则还是用上一次的，如果数字大于路径点的总数，则用终点作为目标点
	  根据角度关系计算alpha，如果此时的速度小于0，则角度为pi-alpha
	  根据公式计算前视距离和delta
	  利用dt和现在的前轮转角更新车辆的状态
	  返回第二步计算，直到迭代时间到达，或者目标追踪点为终点（应该是目标点与最终点的距离小于一个阈值）
'''

import numpy as np
import math
import matplotlib.pyplot as plt


k = 0.3  # look forward gain 利用速度计算前视距离的增益
Lfc = 2.0  # look-ahead distance，防止利用速度进行前视距离计算时候速度为0时前视距离为0
Kp = 1.0  # speed proportional gain,纵向P控制的参数
dt = 0.1  # [s] 时间参数
L = 2.9  # [m] wheel base of vehicle，车身长度，即前后轴的距离


class pure_pursuit:
	def __init__(self,cx,cy,k,Lfc,Kp,dt,L,target_speed):
		self.cx=cx# 跟踪路径
		self.cy=cy
		self.k=k
		self.Lfc=Lfc
		self.Kp=Kp
		self.dt=dt
		self.L=L
		self.target_speed=target_speed

	class State:
		def __init__(self,x,y,yaw,v):
			self.x=x
			self.y=y
			self.yaw=yaw
			self.v=v

		def update(self,delta,a):
			self.x=self.x+self.v*math.cos(self.yaw)*self.dt
			self.y=self.y+self.v*math.sin(self.yaw)*self.dt
			self.yaw=self.yaw+self.v/self.L*math.tan(delta)*self.dt
			self.v=self.v+a*self.dt


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
	def pure_pursuit_control(self,state,pind):
		ind=self.cal_nearest_target_point(state)

	def cal_nearest_target_point(self,state):

		#搜索目标路径上与此事状态最近的点
		dx=[ state.x-i for i in cx]
		dy=[ state.y-i for i in cy]
		dist=[math.hypot(i,j) for (i,j) in zip(cx,cy)]
		nearest_index=dist.index(min(dist))

		#用速度生成前视距离
		ld=self.k*state.v+self.Lfc

		d=0
		ind=nearest_index
		while d<ld and ind<len(self.cx)-1:
			d+=math.hypot(self.cx[ind]-self.cx[ind+1],self.cy[ind]-self.cy[ind+1])
			ind+=1




	def main():
		state=self.state(x,y,yaw,v)#初始状态
		states=self.States(0,state)





if __name__ == '__main__':
	cx = np.arange(0, 50, 0.1)
	cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
	plt.plot(cx,cy,'-b')
	target_speed = 15.0 / 3.6  # [m/s]

	T = 500.0  # max simulation time

	# initial state