import math
import matplotlib.pyplot as plt

# https://www.jianshu.com/p/8b3cdca55dc0
'''
博客里面用来保存点与点之间关系的二维数组其实就是栅格地图，加上motion的代价值，这里两个点之间没有相邻关系的就不能直接进行计算判断，也即是表示了博客中二维数组中
的无穷值
这里我们用一个保存它的父节点的变量parant_index,为了方便我们画出最终的路径
在判断某个点周围的点时，如果经过这个点到周围某个点与起点距离更小，则更新这个周围点的dis值，并且将这个点设为周围点的父节点
dis保存的就是起点到某个点运动的最短距离，如果运动遇见障碍物，则表示不能通过
每次初始的时候都是找dis中找没有被确定为距离起点距离最小的点的 距离最小值，首先把它加入到已经判断为距离起点距离最短的点，然后把它周围所有的点都判断一遍，也即是
判断它所有的出边，然后再重复这步操作
如果在dis中找到没有被确定为距离起点最小的点的距离为最小值，且这个点就是终点，表示找到所需要的路径

算法还有一个verify的步骤，以此来确定起始点和终止点是否存在

'''
class Dijkstra:
	def __init__(self,ox, oy, reso, robot_radius):
		self.robot_radius=robot_radius#机器人的大小，为了能够根据障碍物的坐标得到栅格地图的坐标
		self.reso=reso
		self.motion=[[1, 0, 1.0],
					[0, 1, 1.0],
					[-1, 0, 1.0],
					[0, -1, 1.0],
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
		self.grid_number=(self.width+1)*(self.height+1)#存储的栅格数量，最大值也即为栅格地图的index的最大值
		self.obmap=self.cal_ob_map(ox,oy,robot_radius)
		self.dist,self.openlist,self.closelist=self.param_init()

	def param_init(self):
		dist=[float('inf') for i in range(self.grid_number)]#记录起点到其他点的距离值，会更新
		openlist={}#即未判断的点
		closelist={}#已经判断完成的点，为字典类型，索引值即为他的栅格坐标的index，这样可以通过里面的父节点的index进行索引，最终回溯一条起点到终点的最短路径
		return dist,openlist,closelist

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
		x=float(x_index*self.reso)#应该还需要减去offset
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
		# plt.plot(obx,oby,'*')  #验证栅格地图建立是否正确correct
		# plt.show()
		return obmap

	class Node:
		def __init__(self, x,y,index):#x y 是栅格地图坐标,index为栅格地图索引号
			self.x_index = x  # index of grid,这里是栅格地图的坐标
			self.y_index = y  # index of grid
			self.index=index
			#运动模型 dx dy cost
			self.parent_index=-1


	#https://blog.csdn.net/A_L_A_N/article/details/81392212
	def planning(self,sx, sy, gx, gy):
		print("width=",self.width,"height=",self.height,"x_grid_origin",self.x_grid_origin,"y_grid_origin=",self.y_grid_origin)
		sx,sy=self.cal_grid_xy(sx,sy)#将实际坐标转换为栅格坐标
		gx,gy=self.cal_grid_xy(gx,gy)
		

		nstart=self.Node(sx,sy,self.cal_gridxy_index(sx,sy))#起点的初始化
		nend=self.Node(gx,gy,self.cal_gridxy_index(gx,gy))#终点的初始化
		self.openlist[nstart.index]=nstart#将起点放入已经完成判断的集合
		# print(self.openlist[nstart.index].index,"  1111111")
		self.dist[nstart.index]=0

		while True:

			if self.openlist==None:
				print("openlist is empty")
				break
			min_dist_index=self.find_nearest_to_start()#找到openlist中距离起点最近的点
			# print("min_dist_index=",min_dist_index)
			node_now=self.openlist[min_dist_index]
			self.closelist[min_dist_index]=node_now#将最近的点放到closelist中
			del self.openlist[min_dist_index]#将放到closelist中这个点从openlist中删除

			#每次讲加入到closelist中的点画出来
			plt.plot(node_now.x_index,node_now.y_index, "xc")# 为了能够
			if len(self.closelist.keys()) % 30 == 0:
				plt.pause(0.00001)


			if node_now.x_index==gx and node_now.y_index==gy:
				nend=node_now
				print("find goal")
				break


			for i in range(len(self.motion)):
				node_next=self.Node(node_now.x_index+self.motion[i][0],node_now.y_index+self.motion[i][1],self.cal_gridxy_index(node_now.x_index+self.motion[i][0],node_now.y_index+self.motion[i][1]))
				node_next.parent_index=node_now.index
				# print("i=",i)
				#如果这个点在closelist中，则不进行任何操作
				if self.closelist.get(node_next.index)!=None:
					continue

				#如果这个点是障碍物点，则将这个点加入到closelist中
				if self.obmap[node_next.index]:
					self.closelist[node_next.index]=node_next
					continue

				#如果这个点在openlist中,如果经过node_now的dist更小，则更新dist，并且将node_now设置为node_next的父节点
				if self.openlist.get(node_next.index)!=None:
					if self.dist[node_next.index]>self.dist[node_now.index]+self.motion[i][2]:
						self.dist[node_next.index]=self.dist[node_now.index]+self.motion[i][2]
						self.openlist[node_next.index].parent_index=node_now.index
						continue
					continue
				# 如果这个点不在openlist中
				if self.openlist.get(node_next.index)==None:
					self.openlist[node_next.index]=node_next
					self.dist[node_next.index]=self.dist[node_now.index]+self.motion[i][2]
					# print("add point to openlist")
					continue

				print("cannot find goal")
				break

		rx,ry,cx,cy=self.find_target_way(nend)
		return rx,ry,cx,cy

	#判断此点是否被判断过
	def not_judged(index):
		if self.closelist.get(index)==None:
			return True
		return False


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
		cx=[]
		cy=[]
		node=nend

		while node.parent_index!=-1:
			x,y=self.cal_real_xy(node.x_index,node.y_index)
			rx.append(x)
			ry.append(y)
			node=self.closelist[node.parent_index]

		# 打印出中间路径
		for key in self.closelist:
			x,y=self.cal_real_xy(self.closelist[key].x_index,self.closelist[key].y_index)
			cx.append(x)
			cy.append(y)

		return rx,ry,cx,cy


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
	plt.plot(gx, gy, "xb")
	plt.grid(True)
	plt.axis("equal")
	# plt.show()
	dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)#根据障碍物的坐标值和栅格地图的分辨率，以及机器人的半径，得到一个占据栅格地图,以及obstacle_map
	#index=dijkstra.cal_xy_index(50,-10,dijkstra)
	#print("index=",index)

	#第三个参数是为了内部类调用外部类的属性和函数
	rx, ry,cx,cy = dijkstra.planning(sx, sy, gx, gy)#利用得到的栅格地图，以及起始点和目标点的坐标值，得到生成的路径的xy坐标的list
	plt.plot(rx,ry,"-r")#画出生成的路径
	# plt.plot(cx,cy,'*')
	plt.show()
	
