'''
https://blog.csdn.net/xuzhexing/article/details/90729390
https://blog.csdn.net/weixin_44580210/article/details/90314878
粒子滤波定位可以比单纯地利用观测值更精确
步骤：	1.初始：用大量粒子模拟运动状态，这些粒子在整个运动空间内均匀分布
	    2.预测：根据状态转移方程（运动方程），将每一个粒子带入，得到预测粒子，这里应该包括粒子的速度角速度，以及xy值，进行高维的预测
	    3.校正：对预测粒子进行评价，这里用下一时刻的观测值（有噪声）与预测粒子的距离作评价
	    		距离越短，则对应粒子的权重越大，可以用高斯方程计算距离与对应权重的关系
	    4.重采样：对所有粒子的权重归一化，并进行筛选，既要保留权重大的粒子，又要小部分权重
	    		  小的粒子，具体的方法：
	    		  1.多项式重采样
	    		  2.残差重采样
	    		  3.分层重采样
	    		  4.系统重采样
	    		  重采样带来的新问题是，权值越大的粒子子代越多，相反则子代越少甚至无子代。
	    		  这样重采样后的粒子群多样性减弱，从而不足以用来近似表征后验密度。克服这一
	    		  问题的方法有多种，最简单的就是直接增加足够多的粒子，但这常会导致运算量的
	    		  急剧膨胀。其它方法可以去查看有关文献，这里暂不做介绍。
	    5.更新：用重采样后生成的粒子更新原有的粒子，用这些粒子的位置均值代表粒子滤波的结果，重复步骤2


'''
import numpy as np
import math
import matplotlib.pyplot as plgt

'''
假设		速度的测量方差为0.5
		角速度的测量方差为5度，0.087
		传感器测量rf标志物的误差为0.5m，
'''
def guassian_noise(sigma):#这里用标准差
	y=np.random.randn()*sigma
	return y

v_error=guassian_noise(0.5)
w_error=guassian_noise(0.087)
dist_error=guassian_noise(0.5)
RANGE_DITECT=10#最大探测距离
NP=200
NTh = NP / 2.0  # Number of particle for re-sampling
T_max=100.0
dt=0.1
L=2.5#车长

#用的是最简单的运动方程，根据此时的状态[x y yaw]'，以及速度角速度[u,w]，求出下一时刻的状态
def motion_model(x, u):
	# F = np.array([[1.0, 0, 0],
	# 				[0, 1.0, 0],
	# 				[0, 0, 1.0]])#第四行为0，表示输入的x的最后一个数字没有用，只是为了利于矩阵方程的求解，所以加入了第四列
	# print(x.shape)
	B = np.array([[dt * math.cos(x[2, 0]), 0],
					[dt * math.sin(x[2, 0]), 0],
					[0.0, dt]])

	x = x+ B@u

	return x

def dead_reckoning(x_true,u):
	u[0]+=v_error#给速度和角速度加上高斯噪声
	u[1]+=w_error
	return motion_model(x_true,u)

def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p
#根据上一时刻的真实值，和这一时刻测量得到的速度角速度，得到下一时刻的真实值，以及rf目标的观测值,速度角速度的测量值（加了噪声）
def observation(x_true, u, rf_id):
	x_true_real=motion_model(x_true,u)#根据匀速圆周运动模型获得这一时刻的真实值
	z=np.zeros((0,3))# 0*3的数列，为了后面方便进行堆叠

	for i in range(len(rf_id[:,0])):
		dx=x_true[0,0]-rf_id[i,0]
		dy=x_true[1,0]-rf_id[i,1]
		dist=math.hypot(dx,dy)
		if dist<RANGE_DITECT:
			dist+=dist_error#距离加上噪声，表示传感器测量的有误差
			zi=np.array([[dist,rf_id[i,0],rf_id[i,1]]])
			z=np.vstack(z,zi)
	ud=np.array([[0,0]]).T
	ud[0,0]=u[0,0]+v_error
	ud[1,0]=u[1,0]+w_error


	return x_true_real, z,ud

def re_sampling(px,pw):
	N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number，计算有效粒子数 1/（权值的平方和）
	if N_eff < NTh:#如果有效粒子数太少，则进行重采样
		w_cum = np.cumsum(pw) #每个位置的权值是前面权值和[1,3,5]->[1,4,9]，即为轮盘采样的方法
		base = np.arange(0.0, 1.0, 1 / NP)#返回0.0-1.0步长为1/NP的数列 
		re_sample_id = base + np.random.uniform(0, 1 / NP)#加上噪声，形成均匀分布的随机采样值
		indexes = []
		ind = 0
		for ip in range(NP):
			while re_sample_id[ip] > w_cum[ind]:
				ind += 1
		indexes.append(ind) #存储重采样后的id

		px = px[:, indexes]
		pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

	return px,pw


#输入 粒子群，权重，rf目标观测值，以及测量的速度角速度（有噪声）
def pf_localization(px, pw, z, ud):
	# 预测：根据状态转移方程（运动方程），将每一个粒子带入，得到预测粒子，这里应该包括粒子的速度角速度，以及xy值，进行高维的预测
	for i in range(NP):
		x_pf_tmp=px[:,i] #每一个粒子的状态 x y yaw
		w_tmp=pw[0,i]#每一个粒子的权重
		x_pf_tmp=motion_model(x_pf_tmp,ud)#根据粒子的状态以及测量得到的速度角速度预测粒子的下一个位置
		#根据粒子滤波预测得到的rf距离值和传感器测量的rf距离进行粒子权重的更新？？？？这里的rf的距离真实值无法获取？？？？？？？？？
		for j in range(len(z[0,:])):
			dx=x_pf_tmp[0]-z[j,1]
			dy=x_pf_tmp[1]-z[j,2]
			pre_dist=math.hypot(dx,dy)
			dz=pre_dist-z[j,0]
			w_tmp*=gauss_likelihood(dz, math.sqrt(0.2*0.2))#用预测得到的距离与测量得到的距离的高斯值作为权重
		px[:,i]=x_pf_tmp#更新粒子的状态
		pw[0,i]=w_tmp#更新粒子权重

	#权重归一化
	pw=pw/pw.sum()
	px,pw=re_sampling(px,pw) # 重采样
	x_pf=px@pw.T # 计算得到的粒子滤波的结果。这里感觉应该先进行重采样，与原文作者不一样

	return px,pw,x_pf


def main():
	print(__file__ + " start!!")

	time = 0.0
	
	# RF_ID positions [x, y],用来代替一些已知位置的点
	rf_id = np.array([[10.0, 0.0],
                      [10.0, 10.0],
                      [0.0, 15.0],
                      [-5.0, 20.0]])

	# State Vector [x y yaw]' 第四列只是为了便于进行矩阵计算
	x_pf = np.zeros((3, 1))
	x_true = np.zeros((3, 1))#3*1

	#粒子及其权重的初始化
	px = np.zeros((3, NP))  # Particle store 粒子群，x y yaw各对应一群粒子
	pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight  粒子权重均匀分布
	# x_dr = x_true  # Dead reckoning

	# history
	h_true=np.array([[x_true[0,0],x_true[1,0]]]) # 保存真实值 x y
	# h_dead_reckoning=np.array([[x_true[0],x_true[1]]])#保存航迹推算的值
	h_pf=np.array([[x_pf[0],x_pf[1]]])#保存粒子滤波的结果

	v=1.0 # m/s
	yaw_rate=0.1 # rad/s


	while time<T_max:
		time+=dt
		u=np.array([[v, yaw_rate]]).T#获得转置矩阵 (2,1)矩阵
		x_true_real, z,ud=observation(x_true, u, rf_id)
		# x_dr=dead_reckoning(x_true) #根据上一时刻的真实值和
		px,pw,x_pf=pf_localization(px, pw, z, ud)
		h_true=np.vstack(h_true,np.array([[x_true[0],x_true[1]]])) #保存真实值
		h_pf=np.vstack(h_pf,np.array([[x_pf[0],x_pf[1]]]))


		plt.cla()
		# for stopping simulation with the esc key.
		plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
		plt.plot(h_true[:,0],h_true[:,1],'-g')
		plt.plot(h_pf[:,0],h_pf[:,1],'-b')
		plt.plot(rf_id[:,0],rf_id[:,1],'*r')
		for i in range(len(z[:,0])):
			plt.plot([x_true[0, 0], z[i, 1]], [x_true[1, 0], z[i, 2]], "-k")
		plt.axis("equal")
		plt.grid(True)
		plt.pause(0.001)

	

if __name__ == '__main__':
	main()