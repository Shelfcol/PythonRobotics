import math
import matplotlib.pyplot as plt

	
class AStarPlanner:
	def __init__(self,ox, oy, reso, robot_radius):
		self.robot_radius=robot_radius#机器人的大小，为了能够根据障碍物的坐标得到
		self.reso=reso
		self.motion=[[1, 0, 1],
					[0, 1, 1],
					[-1, 0, 1],
					[0, -1, 1],
					[-1, -1, math.sqrt(2)],
					[-1, 1, math.sqrt(2)],
					[1, -1, math.sqrt(2)],
					[1, 1, math.sqrt(2)]]
		#利用minx和miny为坐标的偏移量，以此计算某个坐标点的索引值
		self.minx=round(min(ox))
		self.maxx=round(max(ox))
		self.miny=round(min(oy))
		self.maxy=round(max(oy))
		self.x_grid_origin=round(self.minx/self.reso)
		self.y_grid_origin=round(self.miny/self.reso)

		self.width=round((self.maxx-self.minx)/self.reso)#x方向上的栅格数
		self.height=round((self.maxy-self.miny)/self.reso)#y方向上的栅格数
		self.obmap=self.cal_ob_map(ox,oy,robot_radius)

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
		return (y_index-self.y_grid_origin)*self.width+x_index-self.x_grid_origin

	def cal_gridxy_index(self,x_index,y_index):
		return (y_index-self.y_grid_origin)*self.width+x_index-self.x_grid_origin

	def cal_real_xy(self,x_index,y_index):
		x=float(x_index*self.reso)
		y=float(y_index*self.reso)
		return x,y


	#建立一个障碍物地图，坐标直接对应某个点的索引值
	def cal_ob_map(self,ox,oy,robot_radius):
		obx=[]
		oby=[]
		obmap=[False for i in range((self.width+1)*(self.height+1))]#这里可能范围不太对，可能会有溢出,所以+1
		for i in range(self.x_grid_origin,self.x_grid_origin+self.width):
			for j in range(self.y_grid_origin,self.y_grid_origin+self.height):
				x_real,y_real=self.cal_real_xy(i,j)

				for x,y in zip(ox,oy):
					if math.hypot(x-x_real,y-y_real)<self.robot_radius:#若格子坐标与障碍物的距离比机器人的半径小，则此格也视为有障碍物，被占据
						real_index=self.cal_gridxy_index(i,j)
						obmap[real_index]=True
						obx.append(i)
						oby.append(j)
		# plt.plot(obx,oby,'*')  #验证栅格地图建立是否正确
		# plt.show()
		return obmap

	#sefl.obmap=cal_ob_map(ox,oy,robot_radius)

	class Node:
		def __init__(self, x,y,gx,gy):#xy，gx gy 是栅格地图坐标
			self.x_index = x  # index of grid,这里是栅格地图的坐标
			self.y_index = y  # index of grid
			self.index=-1
			self.g = 10000 #从初始位置A沿着已生成的路径到待检测的格子的开销
			self.h = self.manhatan_dist(self.x_index,self.y_index,gx,gy)  #待检测格子到目标点的估计移动开销（忽略障碍物)
			self.f=self.g+self.h
			#运动模型 dx dy cost
			self.parent_index=-1
		@staticmethod
		def manhatan_dist(x1,y1,x2,y2):
			# return math.hypot(x1-x2,y1-y2)
			return abs(x1-x2)+abs(y1-y2)

	@staticmethod
	def manhatan_dist(x1,y1,x2,y2):
		# return math.hypot(x1-x2,y1-y2)
		return abs(x1-x2)+abs(y1-y2)

	#https://blog.csdn.net/A_L_A_N/article/details/81392212
	def planning(self,sx, sy, gx, gy):
		print("width=",self.width,"height=",self.height,"x_grid_origin",self.x_grid_origin,"y_grid_origin=",self.y_grid_origin)
		sx,sy=self.cal_grid_xy(sx,sy)#将实际坐标转换为栅格坐标
		gx,gy=self.cal_grid_xy(gx,gy)
		openlist={}
		closelist={}

		nstart=self.Node(sx,sy,gx,gy)
		nstart.index=self.cal_gridxy_index(nstart.x_index,nstart.y_index)
		nstart.g=0
		print(nstart.x_index,nstart.y_index,nstart.index)
		nend=self.Node(gx,gy,gx,gy)
		nend.index=self.cal_gridxy_index(nend.x_index,nend.y_index)

		openlist[self.cal_gridxy_index(sx,sy)]=nstart#将起始点加入到openlis中,key为坐标点对应的index
		print_time=0

		while openlist:
			min_f_key=self.find_smallest_f(openlist)#找到f值最小的点
			# print(min_f_key)
			# print("min_f_key=",openlist[min_f_key].x_index)

			node_now=openlist[min_f_key]
			# if print_time==0:
			# 	print(node_now.index)
			# 	print_time+=1
			# print(node_now.index)
			plt.plot(node_now.x_index,node_now.y_index, "xc")# 为了能够
			if len(closelist.keys()) % 10 == 0:
				plt.pause(0.001)

			closelist[node_now.index]=node_now#加入到closelist中!!!!!!!!!!!!!!
			del openlist[min_f_key]#从开启列表中删除

			#如果目标格在openlist中，则这个点就是终点，则退出循环
			if node_now.x_index==gx and node_now.y_index==gy:
				print("find goal")
				nend=node_now
				break

			#根据运动模型依次判断相邻的点
			# print(len(self.motion))
			for i in range(len(self.motion)):
				node_next=self.Node(node_now.x_index+self.motion[i][0],node_now.y_index+self.motion[i][1],gx,gy)
				node_next.index=self.cal_gridxy_index(node_next.x_index,node_next.y_index)
				node_next.parent_index=node_now.index#将上一个节点作为这个节点的父节点

				#如果这个点障碍物不可通过，do nothing
				if self.obmap[node_next.index]==True:
					continue

				#若这个点在closelist中，do nothing
				if closelist.get(node_next.index)!=None:#字典的get函数，可以防止没有键值对的时候报错
					continue
				#如果这个点不在openlist中，则把它加入到openlist中，并将上一格作为当前格的父节点,计算它的g值
				if openlist.get(node_next.index)==None:
					# node_next.parent_index=node_now.index
					node_next.g=node_now.g+self.motion[i][2]
					openlist[node_next.index]=node_next
				else:#如果此格已经在openlist中
					if openlist.get(node_next.index)!=None:
						#用 g 值为参考检查新的路径是否更好, 更低的g值意味着更好的路径
						# print("222")
						if openlist[node_next.index].g>node_next.g:
							#把这一格的父节点改成当前格, 并且重新计算这一格的g值
							openlist[node_next.index]=node_next#更新
							print("111")
		if openlist==None:
			print("there is no way to target point")

		rx,ry,cx,cy=self.find_target_way(nend,closelist)
		return rx,ry,cx,cy
	@staticmethod
	def find_smallest_f(openlist):
		if openlist==[]:#表示openlist为空,不过这里不会空
			return -1
		min_f=100000000
		key_index=0
		for key in openlist:
			# print("index=",openlist[key].index)
			if openlist[key].f<min_f:
				min_f=openlist[key].f
				key_index=key
		return key_index


	def find_target_way(self,nend,closelist):
		rx=[]
		ry=[]
		cx=[]
		cy=[]
		node=nend
		while node.parent_index!=-1:
			x,y=self.cal_real_xy(node.x_index,node.y_index)
			rx.append(x)
			ry.append(y)
			node=closelist[node.parent_index]

		for key in closelist:
			x,y=self.cal_real_xy(closelist[key].x_index,closelist[key].y_index)
			cx.append(x)
			cy.append(y)

		return rx,ry,cx,cy


if __name__ == '__main__':
	  # start and goal position
	sx = 10.0  # [m]
	sy = 10.0 # [m]
	gx = 22.0  # [m]
	gy = 0.0	 # [m]
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


	plt.plot(ox, oy, ".k")
	plt.plot(sx, sy, "og")
	plt.plot(gx, gy, "xb")
	plt.grid(True)
	plt.axis("equal")
	#plt.show()
	a_star = AStarPlanner(ox, oy, grid_size, robot_radius)#根据障碍物的坐标值和栅格地图的分辨率，以及机器人的半径，得到一个占据栅格地图,以及obstacle_map
	#index=a_star.cal_xy_index(50,-10,a_star)
	#print("index=",index)

	#第三个参数是为了内部类调用外部类的属性和函数
	rx, ry,cx,cy = a_star.planning(sx, sy, gx, gy)#利用得到的栅格地图，以及起始点和目标点的坐标值，得到生成的路径的xy坐标的list
	plt.plot(rx,ry,"-r")#画出生成的路径
	# plt.plot(cx,cy,'*')
	plt.show()
	