import math
import matplotlib.pyplot as plt
import random
import time

'''
https://blog.csdn.net/aoyousihaiqiuqihuang/article/details/100147478
https://www.cnblogs.com/yunlongzhang/p/9265956.html
通过随机采样，增加叶子节点的方式，生成一个随机扩展树，当随机树中的叶子节点包含了目标点或进入了目标区域，便可以在随机树中找到一条由树节点组成的从初始点到
目标点的路径。

RRT 是一种基于概率采样的搜索方法，它采用一种特殊的增量方式进行构造，这种方式能迅速缩短一个随机状态点与树的期望距离。该方法的特点是能够快速有效的搜索高维空间，
通过状态空间的随机采样点，把搜索导向空白区域，从而寻找到一条从起始点到目标点的规划路径。它通过对状态空间中的采样点进行碰撞检测，避免了对空间的建模，能够有效的
解决高维空间和复杂约束的路径规划问题。 RRT算法适合解决多自由度机器人在复杂环境下和动态环境中的路径规划问题[8]。与其他的随机路径规划方法相比， RRT 算法更适用于
非完整约束和多自由度的系统中[9]

步骤：
	1.生成一个随机点，有5%的概率为终点，95%的概率在rand_area(整个地图中的点)之间
	2.获取node_list列表中与rnd_node距离最近的点
	3.获得两点之间的间隔一定距离的点序列
	  对生成的新序列进行碰撞检测，如果不会碰到，则将这个点加入到node_list序列中
	  或者：直接对两点进行碰撞检测
	4.如果这个点与终点距离小于expand_dist，且碰撞检测成功，表示得到了最终路径，否则重复这四步
	5.生成最终的路径，并利用贪心策略进行路径平滑，连接q_start和q_goal,如果检测到两个点之间有障碍物，则将q_goal替换为前一个点，直到两个点能连接上（中间无障碍物）为止。
	一旦q_goal被连接上，则生成最终路径

'''

global print_time
print_time=0

class RRT:
	def __init__(self,ox, oy, reso, robot_radius):
		self.robot_radius=robot_radius#机器人的大小，为了能够根据障碍物的坐标得到栅格地图的坐标
		self.reso=reso
		self.max_iter=50000
		#利用minx和miny为坐标的偏移量，以此计算某个坐标点的索引值
		self.minx=round(min(ox))
		self.maxx=round(max(ox))
		self.miny=round(min(oy))
		self.maxy=round(max(oy))
		self.x_grid_origin=round(self.minx/self.reso)
		self.y_grid_origin=round(self.miny/self.reso)
		self.width=round((self.maxx-self.minx)/self.reso)#x方向上的栅格数
		self.height=round((self.maxy-self.miny)/self.reso)#y方向上的栅格数
		self.grid_number=(self.width+1)*(self.height+1)#存储的栅格数量，最大值也即为栅格地图的index的最大值
		self.obmap=self.cal_ob_map(ox,oy,robot_radius)
		self.node_list={}#保存满足条件的顶点，这里还是用栅格地图的index作为key值进行索引
		self.expand_dist=3*self.reso#如果随机点与终点的距离小于这个值，则表示找到路径
		self.extend_length=20*self.reso#每次最大移动距离



	#栅格地图的原点还是原有的坐标值的原点，只是因为需要计算索引值，因此引入x和y方向上的偏移量，来计算索引值
	
	#根据实际的xy坐标值，得到在栅格地图中的坐标得到索引值,索引值左下角为0，从左到右，从下到上
	def cal_grid_xy(self,x,y):
		x_index=round(x/self.reso)
		y_index=round(y/self.reso)
		return x_index,y_index

	#根据实际的xy坐标值，得到在栅格地图中的索引值，左下角为0，从左到右，从下到上
	def cal_xy_index(self,x,y):
		x_index,y_index=self.cal_grid_xy(x,y)
		print("x_index=",x_index,"y_index=",y_index,"x_grid_origin=",self.x_grid_origin,"y_grid_origin=",self.y_grid_origin,"width=",self.width)
		return round((y_index-self.y_grid_origin)*self.width+x_index-self.x_grid_origin)

	def cal_gridxy_index(self,x_index,y_index):
		x_index=round(x_index)
		y_index=round(y_index)
		return round((y_index-self.y_grid_origin)*self.width+x_index-self.x_grid_origin)

	def cal_real_xy(self,x_index,y_index):
		x=float(x_index*self.reso)
		y=float(y_index*self.reso)
		return x,y


	#建立一个障碍物地图，坐标直接对应某个点的索引值,这个障碍物地图是栅格地图坐标！！！！
	def cal_ob_map(self,ox,oy,robot_radius):
		obx=[]
		oby=[]
		obmap=[False for i in range(self.grid_number)]#这里可能范围不太对，可能会有溢出,所以+1
		for i in range(self.x_grid_origin,self.x_grid_origin+self.width):
			for j in range(self.y_grid_origin,self.y_grid_origin+self.height):
				x_real,y_real=self.cal_real_xy(i,j)

				for x,y in zip(ox,oy):
					if math.hypot(x-x_real,y-y_real)<self.robot_radius:#若格子坐标与障碍物的距离比机器人的半径小，则此格也视为有障碍物，被占据
						real_index=self.cal_gridxy_index(i,j)
						obmap[real_index]=True
						obx.append(i)
						oby.append(j)
		plt.plot(obx,oby,'*')  #验证栅格地图建立是否正确correct
		# plt.show()
		return obmap

	class Node:
		def __init__(self, x,y,index):#x y 是栅格地图坐标,index为栅格地图索引号
			self.x_index = round(x)  # index of grid,这里是栅格地图的坐标
			self.y_index = round(y)  # index of grid
			self.index=index
			#运动模型 dx dy cost
			self.parent_index=-1
			self.child_index=[]
			self.cost=0#保存从起点到这个点的代价


	def planning(self,sx, sy, gx, gy):
		print("width=",self.width,"height=",self.height,"x_grid_origin",self.x_grid_origin,"y_grid_origin=",self.y_grid_origin)
		sx,sy=self.cal_grid_xy(sx,sy)#将实际坐标转换为栅格坐标
		gx,gy=self.cal_grid_xy(gx,gy)
		

		nstart=self.Node(sx,sy,self.cal_gridxy_index(sx,sy))#起点的初始化
		nend=self.Node(gx,gy,self.cal_gridxy_index(gx,gy))#终点的初始化
		self.node_list[nstart.index]=nstart#将起点放入node_list中

		for _ in range(self.max_iter):
			rand_node=self.get_random_node()#生成一个随机点，有5%的概率为终点，95%的概率在rand_area(整个地图中的点)之间

			# plt.plot(rand_node.x_index,rand_node.y_index,"^k")
			# plt.pause(0.001)
			# plt.plot(rand_node.x_index,rand_node.y_index,".w")		

			nearest_node=self.get_nearest_node_from_node_list(rand_node)#获取node_list列表中与rnd_node距离最近的点
			if nearest_node==None:
				continue
			# plt.plot(nearest_node.x_index,nearest_node.y_index,"^g")
			# plt.pause(0.001)
			# plt.plot(nearest_node.x_index,nearest_node.y_index,".w")	


			rand_node.parent_index=nearest_node.index

			rand_node=self.collision_check(nearest_node,rand_node)

			if rand_node==None:#表示有障碍物
				continue
			else:	#表示这个点没有障碍物,返回的是一个node


				# plt.plot(rand_node.x_index,rand_node.y_index,"^k")
				# plt.pause(0.001)
				# plt.plot(rand_node.x_index,rand_node.y_index,".w")

				rand_node.parent_index=nearest_node.index
				if self.obmap[rand_node.index]==True:
					continue


				if self.node_list.get(rand_node.index)!=None:#表示这个点已经存在了,双重判断，防止某几个点的父节点形成循环
					continue

				self.node_list[rand_node.index]=rand_node#将这个随机点加入到生成的树中

				x_tmp=[self.node_list[rand_node.index].x_index,self.node_list[rand_node.parent_index].x_index]
				y_tmp=[self.node_list[rand_node.index].y_index,self.node_list[rand_node.parent_index].y_index]
				plt.plot(x_tmp,y_tmp,'-g')
				# plt.pause(0.0001)

				# 如果这个点与终点距离小于expand_dist，且碰撞检测成功，表示得到了最终路径，否则重复这四步
				if self.distance(rand_node.x_index,rand_node.y_index,gx,gy)<self.expand_dist:
					print("find way")

					nend.parent_index=rand_node.index
					break
		
		rx,ry=self.find_target_way(nend)
		plt.plot(rx,ry,"-r")#画出生成的路径

		rx,ry=self.smoothing(rx,ry)
		plt.plot(rx,ry,"-b")#画出生成的路径
		return rx,ry
	#生成最终的路径，并利用贪心策略进行路径平滑，连接q_start和q_goal,如果检测到两个点之间有障碍物，则将q_goal替换为前一个点，直到两个点能连接上（中间无障碍物）
	#为止。一旦q_goal被连接上，则生成最终路径.rx与ry中的第一个为终点，最后一个点为起点
	def smoothing(self,rx,ry):
		rx_smoothed=[]
		ry_smoothed=[]
		start=self.Node(rx[len(rx)-1],ry[len(rx)-1],self.cal_gridxy_index(rx[len(rx)-1],ry[len(rx)-1]))
		rx_smoothed.append(start.x_index)
		ry_smoothed.append(start.y_index)

		self.extend_length=float('inf')#为了碰撞检测中直接检测两个点，而不是截取的一部分点

		k=len(rx)
		while 1:
			if start.x_index==rx[0] and start.y_index==ry[0]:
				break
			for i in range(0,k):

				if k==1:
					rx_smoothed.append(rx[0])
					ry_smoothed.append(ry[0])
					start=self.Node(rx[0],ry[0],self.cal_gridxy_index(rx[0],ry[0]))
					break

				# print(k,i)
				tmp=self.Node(rx[i],ry[i],self.cal_gridxy_index(rx[i],ry[i]))

				# plt.plot(tmp.x_index,tmp.y_index,"^k")
				# plt.pause(0.1)
				# plt.plot(start.x_index,start.y_index,"^k")	
				# plt.pause(0.1)
				check_point=self.collision_check(start,tmp)

				if check_point==None:#表示中间有障碍物
					continue
				else:
					k=i
					rx_smoothed.append(tmp.x_index)
					ry_smoothed.append(tmp.y_index)
					start=self.Node(rx[i],ry[i],self.cal_gridxy_index(rx[i],ry[i]))
					break
			# plt.show()
		return rx_smoothed,ry_smoothed

	#生成一个随机点，有10%的概率为终点，90%的概率在rand_area(整个地图中的点)之间
	def get_random_node(self):

		if random.randint(0,100)>5:
			x=random.randint(self.x_grid_origin,self.x_grid_origin+self.width)
			y=random.randint(self.y_grid_origin,self.y_grid_origin+self.height)
			rand_node=self.Node(x,y,self.cal_gridxy_index(x,y))

		else:
			rand_node=self.Node(gx,gy,self.cal_gridxy_index(gx,gy))
			
		
		return rand_node

	# 获取node_list列表中与rnd_node距离最近的点的index
	def get_nearest_node_from_node_list(self,rand_node):

		if self.node_list.get(rand_node.index)!=None:#表示这个点已经存在了
			return None

		min_dist=float('inf')
		min_key=None
		for key in self.node_list:
			dist=self.distance(self.node_list[key].x_index,self.node_list[key].y_index,rand_node.x_index,rand_node.y_index)
			if dist<=min_dist:
				min_dist=dist
				min_key=key

		return self.node_list[min_key]

	#对两个点连成的线进行碰撞检测,如果有障碍物，则返回None

	def collision_check(self,nearest_node,rand_node):

		d,theta=self.calc_distance_and_angle(nearest_node,rand_node)#返回两者的角度和长度（整数）
		if d<1:#表示两点在一起,这里的0.5只需要比1小即可
			return None

		if d>self.extend_length:
			d=self.extend_length


		# n_expand = math.floor(d / self.reso)#返回小于等于的一个最大整数
		# print("collision check")
		for i in range(math.floor(d)):
			# print("i=",i)
			x_tmp=nearest_node.x_index+float(i)*math.cos(theta)
			y_tmp=nearest_node.y_index+float(i)*math.sin(theta)
			# print("x=",x_tmp," y=",y_tmp)

			node_tmp_index=self.cal_gridxy_index(x_tmp,y_tmp)
			if self.obmap[node_tmp_index]==True:#表示这格有障碍物，返回None
				return None
		new_node=self.Node(nearest_node.x_index+d*math.cos(theta),nearest_node.y_index+d*math.sin(theta),
			self.cal_gridxy_index(nearest_node.x_index+d*math.cos(theta),nearest_node.y_index+d*math.sin(theta)))
		return new_node

	@staticmethod
	def calc_distance_and_angle(from_node, to_node):

		dx = to_node.x_index - from_node.x_index#整数
		dy = to_node.y_index - from_node.y_index#整数

		dx=float(dx)
		dy=float(dy)
		d = math.hypot(dx, dy)
		theta = math.atan2(dy, dx)#对于垂直于x轴的情况也可以处理
		return d, theta

	# @staticmethod
	# def calc_distance_and_angle(from_node, to_node):

	# 	dx = float(to_node.x_index - from_node.x_index)

	# 	dy = float(to_node.y_index - from_node.y_index)

	# 	if abs(dx)<0.1:#垂直于x轴
	# 		return 1.570796,round(dy)#这里的dy可能小于0

	# 	d = math.hypot(dx, dy)
	# 	theta = math.atan2(dy, dx)
	# 	return d, theta


	@staticmethod
	def distance(x1,y1,x2,y2):
		return math.hypot(float(x1-x2),float(y1-y2))

		rx,ry,cx,cy=self.find_target_way(nend)
		return rx,ry,cx,cy

#在未知点openlist中寻找距离起点最近的点
	def find_nearest_to_start(self):
		# if len(self.dist)==0:#表示已经没有未知的点
		# 	return -1
		min_dist=10000000
		min_dist_index=-1
		for key in self.openlist:
			# print("dist=",self.dist[self.openlist[key].index])
			if self.dist[self.openlist[key].index]<min_dist:
				# print("22222  ",self.dist[self.openlist[key].index])
				min_dist_index=self.openlist[key].index
				# print("min_dist_index=",min_dist_index,"           111")
				min_dist=self.dist[self.openlist[key].index]
		# print("min_dist_index=",min_dist_index,"           333")
		return min_dist_index


	def find_target_way(self,nend):
		rx=[]
		ry=[]
		node=nend
		while node.parent_index!=-1:

			x,y=self.cal_real_xy(node.x_index,node.y_index)
			rx.append(x)
			ry.append(y)

			node=self.node_list[node.parent_index]
			# print(node.parent_index)
		rx.append(node.x_index)#起点
		ry.append(node.y_index)
		return rx,ry


if __name__ == '__main__':
	  # start and goal position
	sx = 10.0  # [m]
	sy = 10.0 # [m]
	gx = 55.0  # [m]
	gy = 55.0	 # [m]
	grid_size = 1.0  # [m]
	robot_radius = 2.0  # [m]

    # set obstable positions
	ox, oy = [], []
	for i in range(-10, 60):
		ox.append(i)
		oy.append(-10.0)
	for i in range(-10, 60):
		ox.append(60.0)
		oy.append(i)
	for i in range(-10, 61):
		ox.append(i)
		oy.append(60.0)
	for i in range(-10, 61):
		ox.append(-10.0)
		oy.append(i)
	for i in range(-10, 40):
		ox.append(20.0)
		oy.append(i)
	for i in range(0, 40):
		ox.append(40.0)
		oy.append(60 - i)


	for i in range(30,50):
		ox.append(i)
		oy.append(20)
	for i in range(50,60):
		ox.append(i)
		oy.append(40)


	plt.plot(ox, oy, ".k")
	plt.plot(sx, sy, "og")
	plt.plot(gx, gy, "^b")
	plt.grid(True)
	plt.axis("equal")
	# plt.show()
	rrt = RRT(ox, oy, grid_size, robot_radius)#根据障碍物的坐标值和栅格地图的分辨率，以及机器人的半径，得到一个占据栅格地图,以及obstacle_map
	#index=dijkstra.cal_xy_index(50,-10,dijkstra)
	#print("index=",index)

	#第三个参数是为了内部类调用外部类的属性和函数
	time_start=time.time()
	rx, ry = rrt.planning(sx, sy, gx, gy)#利用得到的栅格地图，以及起始点和目标点的坐标值，得到生成的路径的xy坐标的list
	# plt.plot(cx,cy,'*')
	time_end=time.time()
	print('time cost',time_end-time_start,'s')
	plt.show()
	