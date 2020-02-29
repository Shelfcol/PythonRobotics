import math
import matplotlib.pyplot as plt

	
class Node:
	def __init__(self, x,y,gx,gy):
		self.x = x  # index of grid
		self.y = y  # index of grid
		self.g = 10000 #从初始位置A沿着已生成的路径到待检测的格子的开销
		self.h = manhatan_disatnce(self.x,self.y,gx,gy)  #待检测格子到目标点的估计移动开销（忽略障碍物)
		self.f=self.g+self.h
		#运动模型 dx dy cost
		self.motion=[[1, 0, 1],
					[0, 1, 1],
					[-1, 0, 1],
					[0, -1, 1],
					[-1, -1, math.sqrt(2)],
					[-1, 1, math.sqrt(2)],
					[1, -1, math.sqrt(2)],
					[1, 1, math.sqrt(2)]]
		self.parent_x=10000
		self.parent_y=10000



#建立一个障碍物地图，坐标直接对应某个点
def cal_ob_map(ox,oy):
	minx = min(ox)
	miny = min(oy)
	maxx = max(ox)
	maxy = max(oy)
	obmap = [[False for i in range(minx-10,maxx+10)]#初始化一个二维数组为False
					for i in range(miny-10,maxy+10)]
	for j in range(len(ox)):
		obmap[ox[j]][oy[j]]=True#表示这个地方是障碍物

	return obmap

#生成openlist和closelist的二维数组，用来保存相应的坐标值是否在其中，若在其中，则为True
def cal_open_close_list(ox,oy):
	minx = min(ox)
	miny = min(oy)
	maxx = max(ox)
	maxy = max(oy)
	openlist = [[False for i in range(minx-10,maxx+10)]#初始化一个二维数组为False
					for i in range(miny-10,maxy+10)]
	closelist = [[False for i in range(minx-10,maxx+10)]#初始化一个二维数组为False
					for i in range(miny-10,maxy+10)]
	return openlist,closelist

#是障碍物，则输出True
def is_obstacle(x,y,obmap):
	return obmap[x][y]
		
def manhatan_disatnce(x,y,gx,gy):
	return abs(x-gx)+abs(y-gy)


def is_in_openlist(x,y,openlist):
	return openlist[x][y]

def is_in_closelist(x,y,closelist):
	return closelist[x][y]

def is_openlist_empty(openlist,ox,oy):
	minx = min(ox)
	miny = min(oy)
	maxx = max(ox)
	maxy = max(oy)
	for i in range(minx-10,maxx+10):
		for j in range(miny-10,maxy+10):
			if openlist[i][j]==True:
				return False
	return True

def search_smallest_f_in_openlist(node_list):
	node=node_list[0]
	index=0
	for i in range(1,len(node_list)):
		if node_list[i].f<node.f:
			node=node_list[i]
			index=i
	return index



def astar_planning(ox,oy,sx, sy, gx, gy):
	openlist,closelist=cal_open_close_list(ox,oy)

	#将障碍物放到closelist中
	for i in range(len(ox)):
		closelist[ox[i]][oy[i]]=True

	openlist[sx][sy]=True#将起始点放到openlist中
	nstart=Node(sx,sy,gx,gy)
	nstart.g=0


	node_list=[]#保存生成的node节点
	node_list.append(nstart)




	while is_openlist_empty(openlist,ox,oy)==False:#当openlist非空的时候
		small_index=search_smallest_f_in_openlist(openlist)
		if openlist[small_index].x==gx and openlist[small_index].y==gy:
			print("find goal")
			break

		node_now=openlist.pop(small_index)#提取f值最小的节点，并删除这个节点
		for i in range(8):
			new_node=Node(node_now.x+node_now.motion[i][0],node_now.y+node_now.motion[i][1],gx,gy)
			new_node.parent_x=node_now.x
			new_node.parent_y=node_now.y
			if new_node.g




if __name__ == '__main__':
	  # start and goal position
	sx = 10  # [m]
	sy = 10  # [m]
	gx = 50  # [m]
	gy = 50	 # [m]
	
    # set obstable positions
	ox, oy = [], []
	for i in range(-10, 60):
		ox.append(i)
		oy.append(-10)
	for i in range(-10, 60):
		ox.append(60)
		oy.append(i)
	for i in range(-10, 61):
		ox.append(i)
		oy.append(60)
	for i in range(-10, 61):
		ox.append(-10)
		oy.append(i)
	for i in range(-10, 40):
		ox.append(20)
		oy.append(i)
	for i in range(0, 40):
		ox.append(40)
		oy.append(60 - i)


	plt.plot(ox, oy, ".k")
	plt.plot(sx, sy, "og")
	plt.plot(gx, gy, "xb")
	plt.grid(True)
	plt.axis("equal")
	#plt.show()
	obmap=cal_ob_map(ox,oy)
	astar_planning(ox,oy,sx, sy, gx, gy)
