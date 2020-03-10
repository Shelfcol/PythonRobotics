#https://blog.csdn.net/ShenDW818/article/details/88669209?utm_source=distribute.pc_relevant.none-task
#https://zhuanlan.zhihu.com/p/106155534有部分有问题 dy没有取绝对值
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
import numpy as np
import math
#从pyplot导入MultipleLocator类，这个类用于设置刻度间隔

#保存xy坐标点的list全局变量
global X_all
global Y_all

X_all=[]
Y_all=[]



def int_list(x):
	y=[]
	for i in x:
		y.append(int(i))
	return y


#根据两个点即直线宽度，，获取线段的四个点，输入参数为int型变量
def get_four_point(x0,y0,x1,y1,width):
	if width<2:
		width=1
	width/=2
	#保存四个点
	X=[]
	Y=[]
	'''
	X.append(x0)
	Y.append(y0)
	X.append(x1)
	Y.append(y1)
	'''

	#逆时针
	if x0==x1:
		if y0==y1:
			X.append(x0)
			Y.append(y0)
			print("!!!!!!!!!!!!!1only one point")
			return X,Y
		
		X.append(x0-width)
		Y.append(y0)
		X.append(x1-width)
		Y.append(y1)

		X.append(x1+width)
		Y.append(y1)

		X.append(x0+width)
		Y.append(y0)
		return X,Y

	#按旋转顺序
	if y0==y1:
		if x0!=x1:
			
			X.append(x0)
			Y.append(y0+width)
			X.append(x1)
			Y.append(y1+width)

			X.append(x1)
			Y.append(y1-width)
			X.append(x0)
			Y.append(y0-width)
			return X,Y			
	#不平行x轴和y轴的讨论

	#按顺时针或者逆时针顺序放入
	x0=float(x0)
	y0=float(y0)
	x1=float(x1)
	y1=float(y1)
	width=float(width)
	# print(x0,y0,x1,y1,width)
	delta=width/(1+(x0-x1)*(x0-x1)/((y1-y0)*(y1-y0)))**0.5
	x2_1=x0+delta
	y2_1=delta*(x0-x1)/(y1-y0)+y0
	x2_2=x0-delta
	y2_2=-delta*(x0-x1)/(y1-y0)+y0

	X.append(x2_1)
	Y.append(y2_1)
	X.append(x2_2)
	Y.append(y2_2)

	tmp_x=x0
	x0=x1
	x1=tmp_x

	tmp_y=y0
	y0=y1
	y1=tmp_y

	delta=width/(1+(x0-x1)*(x0-x1)/((y1-y0)*(y1-y0)))**0.5
	x2_1=x0+delta
	y2_1=delta*(x0-x1)/(y1-y0)+y0
	x2_2=x0-delta
	y2_2=-delta*(x0-x1)/(y1-y0)+y0

	
	X.append(x2_2)
	Y.append(y2_2)
	X.append(x2_1)
	Y.append(y2_1)
	X=int_list(X)
	Y=int_list(Y)
	# print(X,Y)

	return X,Y

def is_in_list(x0,y0,X,Y):
	for i in range(len(X)):
		if x0==X[i]:
			if y0==Y[i]:
				return True
	return False

#种子填充算法，获取矩形区域内的点，输入参数必须在四连通方向封闭
def seed_filling_algorithm(X,Y):
	x_offset=[1,0,-1,0]
	y_offset=[0,1,0,-1]

	x_start=int(np.mean(X))
	y_start=int(np.mean(Y))
	print("x_start=",x_start,"y_start=",y_start)
	# flood_seed_algorithm(x_start,y_start,X,Y,x_offset,y_offset)
	#flood_seed_algorithm_easy_stack(x_start,y_start,X,Y,x_offset,y_offset)
	X,Y=flood_seed_algorithm_scan_stack(x_start,y_start,X,Y)
	return X,Y

def seed_filling_circle(R):
	X,Y=bresenham_circle(R)
	X,Y=flood_seed_algorithm_scan_stack(0,0,X,Y)
	plt_show(X,Y)
#边界填充算法  递归版，不行，会耗尽内存
def flood_seed_algorithm_recursion(x,y,X,Y,x_offset,y_offset):
		if is_in_list(x,y,X,Y)==False:#不是边界点
			X_all.append(x)
			Y_all.append(y)
			for i in range(2):
				flood_seed_algorithm_recursion(x+x_offset[i],y+y_offset[i],X,Y,x_offset,y_offset)

class Stack(object):
    """栈"""
    def __init__(self):
         self.items = []
 
    def is_empty(self):
        """判断是否为空"""
        return self.items == []
 
    def push(self, item):
        """加入元素"""
        self.items.append(item)
 
    def pop(self):
        """弹出元素"""
        return self.items.pop()
 
    def peek(self):
        """返回栈顶元素"""
        return self.items[len(self.items)-1]
 
    def size(self):
        """返回栈的大小"""
        return len(self.items)
    def print(self):
    	for i in self.items:
    		print(i,end=" ")

global print_time
print_time=0

#边界填充算法 简单的栈实现
def flood_seed_algorithm_easy_stack(x,y,X,Y,x_offset,y_offset):
	stack_x=Stack()#保存坐标点x的栈
	stack_y=Stack()#保存坐标点y的栈
	stack_x.push(x)
	stack_y.push(y)
	global print_time
	while stack_x.is_empty()==False:
		x_tmp=stack_x.pop()
		y_tmp=stack_y.pop()
		X_all.append(x_tmp)
		Y_all.append(y_tmp)
		for i in range(len(x_offset)):
			if is_in_list(x_tmp+x_offset[i],y_tmp+y_offset[i],X,Y)==False:
				if print_time<50:
					print("x=",x_tmp+x_offset[i]," y=",y_tmp+y_offset[i])
					print_time+=1
				stack_x.push(x_tmp+x_offset[i])
				stack_y.push(y_tmp+y_offset[i])


#########边界填充算法 边界扫描线算法###########################
#将此点左边的非边界点标记
def getL(x,y,X,Y):
	while is_in_list(x,y,X,Y)==False:
		X.append(x)
		Y.append(y)
		x-=1
	#print("1111111")
	return x+1

def getR(x,y,X,Y):
	while is_in_list(x+1,y,X,Y)==False:
		X.append(x+1)
		Y.append(y)
		x+=1
		#print("222222")

	return x

#获取第y行的最右端边界坐标值
def getNewSeed(stack_x,stack_y,x_left,x_right,y,X,Y):
	tmp_x=x_left+1
	while tmp_x<=x_right:
		if is_in_list(tmp_x,y,X,Y)==True:
			if is_in_list(tmp_x-1,y,X,Y)==False:
				stack_x.push(tmp_x-1)
				stack_y.push(y)
				# print("new seed1 x=",tmp_x-1," y=",y)
		tmp_x+=1
	if is_in_list(tmp_x-1,y,X,Y)==False:#xright处的点也不是边界点
		stack_x.push(tmp_x-1)
		stack_y.push(y)
		# print("new seed2 x=",tmp_x-1," y=",y)
	return stack_x,stack_y

#边界填充算法 边界扫描线算法
def flood_seed_algorithm_scan_stack(x,y,X,Y):
	global print_time
	stack_x=Stack()#保存坐标点x的栈
	stack_y=Stack()#保存坐标点y的栈
	stack_x.push(x)
	stack_y.push(y)
	# global print_time
	while stack_x.is_empty()==False:

		x_tmp=stack_x.pop()
		y_tmp=stack_y.pop()
		#print("x=",x_tmp," y=",y_tmp)
		x_left=getL(x_tmp,y_tmp,X,Y)
		#print("x_left=",x_left)
		x_right=getR(x_tmp,y_tmp,X,Y)
		#plt_show(X_all,Y_all)
		print("x_left=",x_left,"x_right=",x_right,"y=",y_tmp)
		stack_x,stack_y=getNewSeed(stack_x,stack_y,x_left,x_right,y_tmp-1,X,Y)
		stack_x,stack_y=getNewSeed(stack_x,stack_y,x_left,x_right,y_tmp+1,X,Y)
	return X,Y
	#plt_show(X,Y)

#####################################################

#画指定宽度的直线
def bresenham_width(x0,y0,x1,y1,width):
	X4,Y4=get_four_point(x0,y0,x1,y1,width)
	X4.append(X4[0])
	Y4.append(Y4[0])
	# print("222")
	# print(X4,Y4)
	X=[]
	Y=[]
	for i in range(4):
		#print("i=",i)
		#print(X4[i],Y4[i],X4[i+1],Y4[i+1])
		x,y=bresenham_check(X4[i],Y4[i],X4[i+1],Y4[i+1])
		print(x,y)
		for xi in x:
			X.append(xi)
			#X_all.append(xi)
		for yi in y:
			Y.append(yi)
			#Y_all.append(yi)
	#plt_show(X,Y)
	print("11111")
	
	X,Y=seed_filling_algorithm(X,Y)
	plt_show(X,Y)

#https://blog.csdn.net/sinat_41104353/article/details/82961824
def bresenham_circle(R):
	r=round(R)
	x=0
	y=r
	p=3-2*r
	X=[]
	Y=[]
	while x<=y:


		X.append(x)#2
		Y.append(y)
		#画另外七个对称点
		X.append(y)#1
		Y.append(x)	

		X.append(-x)#3
		Y.append(y)

		X.append(-y)#4
		Y.append(x)

		X.append(-y)#5
		Y.append(-x)

		X.append(-x)#6
		Y.append(-y)

		X.append(x)#7
		Y.append(-y)

		X.append(y)#8
		Y.append(-x)

		if p>=0:
			p+=4*(x-y)+10
			y-=1
			x+=1
		else:
			p+=4*x+6
			x+=1

	# x_c=np.linspace(0,r,100)
	# for i in range(len(x_c)):
	# 	y_c=math.sqrt(r*r-x_c[i]*x_c[i])
	# 	if y_c<x_c[i]:
	# 		break
	# 	X.append(x_c[i])
	# 	Y.append(y_c)

	# plt_show(X,Y)
	return X,Y






def bresenham_k_lessthan_1(x0,y0,x1,y1):
	x0=round(x0)
	x1=round(x1)
	y0=round(y0)
	y1=round(y1)

	dx=x1-x0
	dy=y1-y0


	x=[]
	y=[]
	x.append(x0)
	y.append(y0)

	p=2*dy-dx
	xi=x0
	yi=y0

	for xi in range(x0,x1+1):
		x.append(xi)
		y.append(yi)

		#calculate next p
		if p>0:
			p=p+2*(dy-dx)
			yi=yi+1
	
		else:
			p=p+2*dy

	# plt_show(x,y)
	return x,y


def bresenham_check(x0,y0,x1,y1):
	#将直线朝向为二三象限的点全部转换到一四象限
	if x0>x1:
		tmp=x0
		x0=x1
		x1=tmp
		tmp=y1
		y1=y0
		y0=tmp	
	dy=y1-y0
	dx=x1-x0
	if dy>0:#表示一象限
		if abs(dx)>abs(dy):#第一象限斜率小于1部分
			x,y= bresenham_k_lessthan_1(x0,y0,x1,y1)
			# plt_show(x,y)
			return x,y
		else:
			#xy值互换
			y,x=bresenham_k_lessthan_1(y0,x0,y1,x1)
			# plt_show(x,y)
			return x,y
	else:#表示第四象限 dy<0
		if abs(dx)>abs(dy):
			x,y=bresenham_k_lessthan_1(x0,-1*y0,x1,-1*y1)
			# print(len(x),len(y))
			for i in range(len(y)):
				y[i]=-1*y[i]

			# plt_show(x,y)
			return x,y
		else:
			y,x=bresenham_k_lessthan_1(-1*y0,x0,-1*y1,x1)
			for i in range(len(y)):
				y[i]=-1*y[i]

			# plt_show(x,y)
			return x,y

def plt_show(x,y):
	'''
	#画出首末两点练成的直线
	if x[0]!=x[-1]:
		X=np.linspace(x[0],x[-1],100)
		Y=(y[-1]-y[0])/(x[-1]-x[0])*(X-x[0])+y[0]
		plt.plot(X,Y)
	'''

	plt.scatter(x, y, s=200, label = '$like$', c = 'blue', marker='s', alpha = None, edgecolors= 'white')
	# 用scatter绘制散点图,可调用marker修改点型, label图例用$$包起来表示使用latex公式编辑显示，写\sum符号效果比较明显，普通的符号带上后就变成了斜体，edgecolors边缘颜色，只有前两个是必备参数
	plt.legend()
	plt.xlabel('x')
	plt.ylabel('y')
	plt.axis('equal')
	plt.grid()


	x_major_locator=MultipleLocator(1)
	#把x轴的刻度间隔设置为1，并存在变量里
	y_major_locator=MultipleLocator(1)
	#把y轴的刻度间隔设置为10，并存在变量里
	ax=plt.gca()
	#ax为两条坐标轴的实例
	ax.xaxis.set_major_locator(x_major_locator)
	#把x轴的主刻度设置为1的倍数
	ax.yaxis.set_major_locator(y_major_locator)
	#把y轴的主刻度设置为10的倍数
	#plt.xlim([x[0],x[-1]])
	#把x轴的刻度范围设置为-0.5到11，因为0.5不满一个刻度间隔，所以数字不会显示出来，但是能看到一点空白
	#plt.ylim(-5,110)
	#把y轴的刻度范围设置为-5到110，同理，-5不会标出来，但是能看到一点空白



	plt.show()

if "__name==__main__":
	# bresenham_check(0,0,10,-1)#都可以√√√√√√
	# bresenham_circle(8)
	# bresenham_width(0,0,100,100,8)
	seed_filling_circle(30)